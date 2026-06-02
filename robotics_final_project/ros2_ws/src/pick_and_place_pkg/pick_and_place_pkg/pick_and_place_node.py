import time
import threading
import math
import os
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface
from custom_interface.srv import MoveRobot
from pick_and_place_pkg.constants import (
    SQUARE_COORDS_M, BOARD_Z, HOVER_ABOVE_BOARD_M, CENTER_HOVER_ABOVE_BOARD_M,
    PLACE_Z_OFFSET, TOP_DOWN, SOLENOID_OFF_DELAY_DEFAULT,
    make_pose, square_center_in_world, compute_board_center
)
from pick_and_place_pkg.arduino_client import ArduinoSerialClient, SERIAL_AVAILABLE, list_ports

# from std_msgs.msg import String  # Uncomment when button integration is ready


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place_node")
        self.move_service = self.create_service(MoveRobot, "move_robot", self.handle_move_robot)
        self.setup_parameters()
        self.setup_moveit()
        self.setup_gripper()
        self.setup_arduino()
        # self.setup_button_publisher()  # Uncomment when button integration is ready
        self.get_logger().info("PickAndPlaceNode is ready.")

    def setup_parameters(self):
        self.declare_parameter("cartesian_fraction_threshold", 0.90)
        self.declare_parameter("arduino_enabled", True)
        self.declare_parameter("arduino_port", "/dev/ttyUSB0")
        self.declare_parameter("arduino_baud", 115200)
        self.declare_parameter("solenoid_off_delay", float(SOLENOID_OFF_DELAY_DEFAULT))
        self.declare_parameter("auto_detect_arduino", True)
        self.declare_parameter("task", "serve")
        self.declare_parameter("tune_mode", "center")
        self.declare_parameter("tune_square", "b3")
        self.j_home = [0.0] * 6
        self.gripper_closed = False
        self.gripper_locked = False

    def setup_moveit(self):
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

    def setup_gripper(self):
        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.8],
                closed_gripper_joint_positions=[0.0],
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=True,
            )
            self.get_logger().info("GripperInterface initialized.")
        except Exception as e:
            self.gripper = None
            self.get_logger().warn(f"Gripper interface init failed: {e}")

    def setup_arduino(self):
        self.arduino = None
        try:
            enabled = self.get_parameter("arduino_enabled").get_parameter_value().bool_value
            port_param = self.get_parameter("arduino_port").get_parameter_value().string_value
            baud = self.get_parameter("arduino_baud").get_parameter_value().integer_value
            autod = self.get_parameter("auto_detect_arduino").get_parameter_value().bool_value

            chosen_port = port_param

            if enabled and autod and SERIAL_AVAILABLE and list_ports is not None:
                try:
                    byid_dir = "/dev/serial/by-id"
                    if os.path.isdir(byid_dir):
                        entries = sorted(os.listdir(byid_dir))
                        if entries:
                            chosen_port = os.path.join(byid_dir, entries[0])
                            self.get_logger().info(f"Auto-detected Arduino by-id: {chosen_port}")

                    if chosen_port == port_param:
                        for p in list_ports.comports():
                            desc = (p.description or "").lower()
                            if "arduino" in desc or "usb serial" in desc or "ch341" in desc or "ftdi" in desc:
                                chosen_port = p.device
                                self.get_logger().info(f"Auto-detected Arduino: {chosen_port}")
                                break
                except Exception as e:
                    self.get_logger().debug(f"Auto-detect failed: {e}")

            if enabled:
                self.arduino = ArduinoSerialClient(
                    node=self,
                    port=chosen_port,
                    baud=baud,
                    # button_callback=self.on_button_pressed  # Uncomment when button integration is ready
                )
                self.get_logger().info(f"Arduino client enabled on {chosen_port} @ {baud}")
            else:
                self.get_logger().info("Arduino disabled via parameter.")

        except Exception as e:
            self.get_logger().warn(f"Failed to initialize Arduino client: {e}")
            self.arduino = None

    # def setup_button_publisher(self):
    #     self.button_pub = self.create_publisher(String, "button_feed", 10)
    #     self.get_logger().info("button_feed publisher created.")

    # def on_button_pressed(self):
    #     try:
    #         from std_msgs.msg import String
    #         msg = String()
    #         msg.data = "BUTTON_PRESSED"
    #         self.button_pub.publish(msg)
    #         self.get_logger().info("Published button press to /button_feed")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to publish button event: {e}")

    def wait_for_moveit(self, timeout_s=60):
        self.get_logger().info(f"Waiting up to {timeout_s}s for planner and joint states...")

        start = time.time()
        planner_ready = False

        while time.time() - start < timeout_s:
            try:
                out = subprocess.run(
                    ["ros2", "service", "list"],
                    capture_output=True, text=True, timeout=3
                ).stdout
            except subprocess.TimeoutExpired:
                out = ""

            if "/plan_kinematic_path" in out:
                planner_ready = True
                break

            time.sleep(0.5)

        if not planner_ready:
            self.get_logger().warn("Timed out waiting for planner service.")

        expected_joints = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        joints_ready = False
        start2 = time.time()

        while time.time() - start2 < timeout_s:
            try:
                joint_out = subprocess.run(
                    ["ros2", "topic", "echo", "/joint_states", "--once"],
                    capture_output=True, text=True, timeout=3
                ).stdout
            except subprocess.TimeoutExpired:
                joint_out = ""

            if "name:" in joint_out and "position:" in joint_out:
                if all(j in joint_out for j in expected_joints):
                    joints_ready = True
                    break

            time.sleep(0.3)

        if not joints_ready:
            self.get_logger().warn("Timed out waiting for joint states.")

        ready = planner_ready and joints_ready
        if ready:
            self.get_logger().info("Planner and joint states ready.")

        return ready

    def move_to_joints(self, joints) -> bool:
        try:
            self.get_logger().info(f"Moving to joints: {[f'{p:.3f}' for p in joints]}")
            self.moveit2.move_to_configuration(joint_positions=joints)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"move_to_joints failed: {e}")
            return False

    def plan_and_execute_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        try:
            threshold = float(self.get_parameter("cartesian_fraction_threshold").get_parameter_value().double_value)
        except Exception:
            threshold = 0.90

        if threshold <= 0.0 or threshold > 1.0:
            threshold = 0.90

        try:
            traj = self.moveit2.plan(
                pose=pose,
                cartesian=cartesian,
                max_step=0.005,
                cartesian_fraction_threshold=(threshold if cartesian else None),
            )
        except Exception as e:
            self.get_logger().warn(f"Planning exception: {e}")
            return False

        if traj is None:
            self.get_logger().warn(f"Planning failed. Target: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            return False

        try:
            self.moveit2.execute(traj)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().warn(f"Execution failed: {e}")
            return False

    def open_gripper(self):
        if self.gripper_locked:
            self.get_logger().warn("Cannot open gripper: gripper is locked.")
            return False
        if self.gripper is None:
            self.get_logger().warn("Cannot open gripper: gripper interface is missing.")
            return False
        try:
            self.gripper.open()
            time.sleep(0.15)
            self.gripper_closed = False
            return True
        except Exception as e:
            self.get_logger().warn(f"Gripper open failed: {e}")
            return False

    def close_gripper(self):
        if self.gripper is None:
            self.get_logger().warn("Cannot close gripper: gripper interface is missing.")
            self.gripper_closed = True
            return False
        try:
            self.gripper.close()
            time.sleep(0.15)
            self.gripper_closed = True
            return True
        except Exception as e:
            self.get_logger().warn(f"Gripper close failed: {e}")
            return False

    def lock_gripper(self):
        if self.gripper is None:
            self.get_logger().warn("Cannot lock gripper: gripper interface is missing.")
            return

        def blocked_open(*args, **kwargs):
            self.get_logger().warn("Gripper open blocked — gripper is locked.")
            return False

        try:
            if not hasattr(self.gripper, "original_open"):
                self.gripper.original_open = getattr(self.gripper, "open", None)
            self.gripper.open = blocked_open
            self.gripper_locked = True
            self.get_logger().info("Gripper locked successfully.")
        except Exception as e:
            self.get_logger().warn(f"Failed to lock gripper: {e}")

    def activate_solenoid(self):
        try:
            if self.arduino:
                ok = self.arduino.send_command("sol on")
                if ok:
                    self.get_logger().info("Solenoid activated.")
                else:
                    self.get_logger().warn("Solenoid activation failed.")
        except Exception as e:
            self.get_logger().warn(f"Solenoid activation error: {e}")

    def deactivate_solenoid(self):
        try:
            if self.arduino:
                ok = self.arduino.send_command("sol off")
                if ok:
                    self.get_logger().info("Solenoid deactivated.")
                else:
                    self.get_logger().warn("Solenoid deactivation failed.")
        except Exception as e:
            self.get_logger().warn(f"Solenoid deactivation error: {e}")

        try:
            sol_off_delay = float(self.get_parameter("solenoid_off_delay").get_parameter_value().double_value)
        except Exception:
            sol_off_delay = SOLENOID_OFF_DELAY_DEFAULT

        if sol_off_delay > 0.0:
            time.sleep(sol_off_delay)

    def move_piece(self, src: str, dst: str) -> bool:
        src = src.lower()
        dst = dst.lower()

        if src not in SQUARE_COORDS_M or dst not in SQUARE_COORDS_M:
            self.get_logger().error(f"Invalid square passed to move_piece: {src} -> {dst}")
            return False

        source_center = square_center_in_world(src)
        target_center = square_center_in_world(dst)

        hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
        pick_z = BOARD_Z + PLACE_Z_OFFSET
        place_z = BOARD_Z + PLACE_Z_OFFSET

        source_hover = make_pose(source_center.position.x, source_center.position.y, hover_z, *TOP_DOWN)
        source_pick = make_pose(source_center.position.x, source_center.position.y, pick_z, *TOP_DOWN)
        target_hover = make_pose(target_center.position.x, target_center.position.y, hover_z, *TOP_DOWN)
        target_place = make_pose(target_center.position.x, target_center.position.y, place_z, *TOP_DOWN)

        if not self.plan_and_execute_pose(source_hover, cartesian=False):
            self.get_logger().warn("Hover to source failed — trying offsets.")
            success = False
            for dx, dy in [(0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)]:
                trial = make_pose(source_hover.position.x + dx, source_hover.position.y + dy, source_hover.position.z, *TOP_DOWN)
                if self.plan_and_execute_pose(trial, cartesian=False):
                    success = True
                    break
            if not success:
                self.get_logger().error("Failed to reach source hover.")
                return False

        time.sleep(0.08)
        self.activate_solenoid()

        if not self.plan_and_execute_pose(source_pick, cartesian=True):
            self.get_logger().warn("Direct descent to source failed — trying staged descent.")
            ok_descend = True
            for i in range(1, 4):
                z_step = source_hover.position.z + (source_pick.position.z - source_hover.position.z) * (i / 3)
                step_pose = make_pose(source_pick.position.x, source_pick.position.y, z_step, *TOP_DOWN)
                if not self.plan_and_execute_pose(step_pose, cartesian=True):
                    ok_descend = False
                    break
                time.sleep(0.05)
            if not ok_descend:
                self.get_logger().error("Staged descent to source failed.")
                return False

        if not self.plan_and_execute_pose(source_hover, cartesian=True):
            self.get_logger().warn("Lift from source failed (cartesian) — trying joint space.")
            if not self.plan_and_execute_pose(source_hover, cartesian=False):
                self.get_logger().error("Failed to lift after pick.")
                return False

        time.sleep(0.08)

        dist_xy = math.hypot(
            source_hover.position.x - target_hover.position.x,
            source_hover.position.y - target_hover.position.y
        )
        prefer_cartesian = dist_xy < 0.08

        if prefer_cartesian:
            if not self.plan_and_execute_pose(target_hover, cartesian=True):
                self.get_logger().warn("Cartesian travel to target failed — trying joint space.")
                if not self.plan_and_execute_pose(target_hover, cartesian=False):
                    self.get_logger().error("Failed to travel to target hover.")
                    return False
        else:
            if not self.plan_and_execute_pose(target_hover, cartesian=False):
                self.get_logger().warn("Joint space travel to target failed — trying cartesian.")
                if not self.plan_and_execute_pose(target_hover, cartesian=True):
                    self.get_logger().error("Failed to travel to target hover.")
                    return False

        time.sleep(0.08)

        if not self.plan_and_execute_pose(target_place, cartesian=True):
            self.get_logger().warn("Direct descent to target failed — trying staged descent.")
            ok_place = True
            for i in range(1, 4):
                z_step = target_hover.position.z + (target_place.position.z - target_hover.position.z) * (i / 3)
                step_pose = make_pose(target_place.position.x, target_place.position.y, z_step, *TOP_DOWN)
                if not self.plan_and_execute_pose(step_pose, cartesian=True):
                    ok_place = False
                    break
                time.sleep(0.05)
            if not ok_place:
                self.get_logger().error("Staged descent to target failed.")
                return False

        self.deactivate_solenoid()

        if not self.plan_and_execute_pose(target_hover, cartesian=True):
            self.get_logger().warn("Retract from target failed (cartesian) — trying joint space.")
            self.plan_and_execute_pose(target_hover, cartesian=False)

        return True

    def handle_move_robot(self, request, response):
        uci = request.best_uci.strip().lower()
        self.get_logger().info(f"Received move request: {uci}")

        if len(uci) != 4:
            response.robot_status_message = "ERROR: Invalid UCI"
            return response

        src = uci[:2]
        dst = uci[2:]

        success = self.move_piece(src, dst)
        response.robot_status_message = "SUCCESS" if success else "ERROR"
        return response

    def task_tune(self):
        ready = self.wait_for_moveit(timeout_s=60)
        if not ready:
            self.get_logger().warn("Continuing tuning despite incomplete readiness.")

        tune_mode = self.get_parameter("tune_mode").get_parameter_value().string_value.lower()

        if tune_mode == "home":
            self.get_logger().info("[TUNE] Moving to home position.")
            if not self.move_to_joints(self.j_home):
                self.get_logger().error("[TUNE] Failed to move to home position.")
            return

        if tune_mode == "hold":
            self.get_logger().info("[TUNE] Closing and locking gripper.")
            if not self.close_gripper():
                self.get_logger().warn("[TUNE] Gripper close failed.")
            self.lock_gripper()
            self.get_logger().info("[TUNE] Gripper is now locked.")
            return

        if tune_mode == "open":
            self.get_logger().info("[TUNE] Opening gripper.")
            if not self.open_gripper():
                self.get_logger().warn("[TUNE] Gripper open failed.")
            return

        center_x, center_y, center_z = compute_board_center()
        center_hover_z = center_z + CENTER_HOVER_ABOVE_BOARD_M
        center_hover_pose = make_pose(center_x, center_y, center_hover_z, *TOP_DOWN)

        if tune_mode == "center":
            self.get_logger().info(f"[TUNE] Moving to center hover at x={center_x:.3f}, y={center_y:.3f}, z={center_hover_z:.3f}")
            if not self.plan_and_execute_pose(center_hover_pose, cartesian=False):
                self.get_logger().warn("[TUNE] Joint space move failed — trying cartesian.")
                if not self.plan_and_execute_pose(center_hover_pose, cartesian=True):
                    self.get_logger().error("[TUNE] Failed to reach center hover.")
            else:
                self.get_logger().info("[TUNE] Reached center hover successfully.")
            return

        if tune_mode in ("square", "lift"):
            tune_square = self.get_parameter("tune_square").get_parameter_value().string_value.lower()

            if not tune_square or tune_square not in SQUARE_COORDS_M:
                self.get_logger().error(f"[TUNE] Invalid or missing tune_square: '{tune_square}'.")
                return

            square_center = square_center_in_world(tune_square)
            hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
            square_hover = make_pose(square_center.position.x, square_center.position.y, hover_z, *TOP_DOWN)
            square_pick = make_pose(square_center.position.x, square_center.position.y, BOARD_Z + PLACE_Z_OFFSET, *TOP_DOWN)

            self.get_logger().info(f"[TUNE] Moving to hover above {tune_square} at x={square_hover.position.x:.3f}, y={square_hover.position.y:.3f}, z={square_hover.position.z:.3f}")

            if not self.plan_and_execute_pose(square_hover, cartesian=False):
                self.get_logger().warn("[TUNE] Hover joint move failed — trying offsets.")
                success = False
                for dx, dy in [(0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)]:
                    trial = make_pose(square_hover.position.x + dx, square_hover.position.y + dy, square_hover.position.z, *TOP_DOWN)
                    if self.plan_and_execute_pose(trial, cartesian=False):
                        success = True
                        break
                if not success:
                    if not self.plan_and_execute_pose(square_hover, cartesian=True):
                        self.get_logger().error("[TUNE] Failed to reach hover above square.")
                        return

            time.sleep(0.08)

            if tune_mode == "square":
                self.get_logger().info(f"[TUNE] Descending to board level at {tune_square}.")
                if not self.plan_and_execute_pose(square_pick, cartesian=True):
                    self.get_logger().warn("[TUNE] Direct descent failed — trying staged descent.")
                    ok_descend = True
                    for i in range(1, 5):
                        z_step = square_hover.position.z + (square_pick.position.z - square_hover.position.z) * (i / 4)
                        step_pose = make_pose(square_pick.position.x, square_pick.position.y, z_step, *TOP_DOWN)
                        if not self.plan_and_execute_pose(step_pose, cartesian=True):
                            ok_descend = False
                            break
                        time.sleep(0.05)
                    if not ok_descend:
                        self.get_logger().error("[TUNE] Staged descent failed.")
                        return
                self.get_logger().info(f"[TUNE] Reached board level at {tune_square}.")
                return

            if tune_mode == "lift":
                self.get_logger().info(f"[TUNE] Descending to board level at {tune_square}.")
                if not self.plan_and_execute_pose(square_pick, cartesian=True):
                    self.get_logger().warn("[TUNE] Direct descent failed — trying staged descent.")
                    ok_descend = True
                    for i in range(1, 4):
                        z_step = square_hover.position.z + (square_pick.position.z - square_hover.position.z) * (i / 3)
                        step_pose = make_pose(square_pick.position.x, square_pick.position.y, z_step, *TOP_DOWN)
                        if not self.plan_and_execute_pose(step_pose, cartesian=True):
                            ok_descend = False
                            break
                        time.sleep(0.05)
                    if not ok_descend:
                        self.get_logger().error("[TUNE] Staged descent failed.")
                        return

                time.sleep(0.08)

                self.get_logger().info(f"[TUNE] Lifting back to hover above {tune_square}.")
                if not self.plan_and_execute_pose(square_hover, cartesian=True):
                    self.get_logger().warn("[TUNE] Cartesian lift failed — trying joint space.")
                    if not self.plan_and_execute_pose(square_hover, cartesian=False):
                        self.get_logger().error("[TUNE] Failed to lift back to hover.")
                        return

                self.get_logger().info(f"[TUNE] Lift complete at {tune_square}.")
                return

        self.get_logger().error(f"[TUNE] Unknown tune_mode '{tune_mode}'. Valid modes: home, hold, open, center, square, lift.")

    def destroy_node(self):
        if self.arduino:
            self.arduino.stop_serial_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    task = node.get_parameter("task").get_parameter_value().string_value

    try:
        if task == "tune":
            node.task_tune()
        elif task == "home":
            node.move_to_joints(node.j_home)
        else:
            node.get_logger().info("PickAndPlaceNode serving move_robot requests.")
            threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino:
            node.arduino.stop_serial_connection()
        rclpy.shutdown()


if __name__ == "__main__":
    main()