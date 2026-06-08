import time
import threading
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface
from custom_interface.srv import MoveRobot
from pick_and_place_pkg.constants import (
    SQUARE_COORDS_M, BOARD_Z, HOVER_ABOVE_BOARD_M, CENTER_HOVER_ABOVE_BOARD_M,
    CENTER_HOVER_Z,
    PLACE_Z_OFFSET, TOP_DOWN,
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
        if not self.wait_for_moveit():
            self.get_logger().error("MoveIt not available. Shutting down.")
            raise RuntimeError("MoveIt not available.")
        # self.setup_button_publisher()  # Uncomment when button integration is ready
        self.get_logger().info("PickAndPlaceNode is ready.")

    # -------------------------
    # Setup
    # -------------------------

    def setup_parameters(self):
        self.declare_parameter("arduino_enabled", True)
        self.declare_parameter("arduino_port", "/dev/ttyUSB0")
        self.declare_parameter("arduino_baud", 115200)
        self.declare_parameter("auto_detect_arduino", True)
        self.declare_parameter("task", "serve")
        self.declare_parameter("tune_mode", "center")
        self.declare_parameter("tune_square", "b3")
        self.j_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def setup_moveit(self):
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
            use_move_group_action=True,
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
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")

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
                self.arduino = ArduinoSerialClient(node=self, port=chosen_port, baud=baud)
                self.get_logger().info(f"Arduino client enabled on {chosen_port} @ {baud}")
            else:
                self.get_logger().info("Arduino disabled via parameter.")

        except Exception as e:
            self.get_logger().warn(f"Failed to initialize Arduino client: {e}")
            self.arduino = None

    def wait_for_moveit(self, timeout_s: int = 5) -> bool:
        self.get_logger().info(f"Waiting {timeout_s}s for MoveIt to be ready...")
        start = time.time()
        last_log = start

        while time.time() - start < timeout_s:
            elapsed = time.time() - start

            try:
                result = self.moveit2.compute_fk()
                if result is not None:
                    self.get_logger().info("MoveIt is ready.")
                    return True
            except Exception:
                pass

            if time.time() - last_log >= 5.0:
                self.get_logger().info(f"Still waiting for MoveIt... ({elapsed:.0f}s elapsed)")
                last_log = time.time()

            time.sleep(0.3)

        self.get_logger().warn("Timed out waiting for MoveIt.")
        return False

    # -------------------------
    # Motion primitives
    # -------------------------

    def move_to_joints(self, joints) -> bool:
        try:
            self.get_logger().info(f"Moving to joints: {[f'{p:.3f}' for p in joints]}")
            self.moveit2.move_to_configuration(joint_positions=joints)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"move_to_configuration failed: {e}")
            return False

    def plan_and_execute_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        try:
            traj = self.moveit2.plan(
                pose=pose,
                cartesian=cartesian,
                max_step=0.005,
                cartesian_fraction_threshold=0.6 if cartesian else None,
            )
            if traj is None:
                self.get_logger().error(
                    f"Planning failed (cartesian={cartesian}). "
                    f"Target: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}"
                )
                return False
            self.moveit2.execute(traj)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"plan_and_execute_pose failed: {e}")
            return False

    # -------------------------
    # Gripper
    # -------------------------

    def gripper_control(self, command: str):
        if command not in ("open", "close"):
            self.get_logger().error(f"Invalid gripper command: '{command}'. Use 'open' or 'close'.")
            return
        if self.gripper is None:
            self.get_logger().warn("Gripper command skipped — no gripper interface available.")
            return
        if command == "open":
            self.gripper.open()
        else:
            self.gripper.close()

    # -------------------------
    # Solenoid
    # -------------------------

    def solenoid_control(self, command: str):
        if command not in ("sol on", "sol off"):
            self.get_logger().error(f"Invalid solenoid command: '{command}'. Use 'sol on' or 'sol off'.")
            return
        if self.arduino is None:
            self.get_logger().warn("Solenoid command skipped — no Arduino client available.")
            return
        try:
            ok = self.arduino.send_command(command)
            if ok:
                self.get_logger().info(f"Solenoid: {command}")
            else:
                self.get_logger().warn(f"Solenoid command failed: {command}")
        except Exception as e:
            self.get_logger().warn(f"Solenoid control error: {e}")

        if command == "sol off":
            time.sleep(0.2)

    # -------------------------
    # High level motion
    # -------------------------

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

        if not self.plan_and_execute_pose(source_pick, cartesian=True):
            self.get_logger().error("Descent to source failed.")
            return False

        time.sleep(0.08)
        self.solenoid_control("sol on")

        if not self.plan_and_execute_pose(target_place, cartesian=True):
            self.get_logger().error("Descent to target failed.")
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

        self.solenoid_control("sol off")

        if not self.plan_and_execute_pose(target_hover, cartesian=True):
            self.get_logger().warn("Retract from target failed (cartesian) — trying joint space.")
            self.plan_and_execute_pose(target_hover, cartesian=False)

        return True

    # -------------------------
    # Tasks
    # -------------------------

    def task_tune(self):
        tune_mode = self.get_parameter("tune_mode").get_parameter_value().string_value.lower()

        if tune_mode == "home":
            self.get_logger().info("[TUNE] Moving to home position.")
            if not self.move_to_joints(self.j_home):
                self.get_logger().error("[TUNE] Failed to move to home position.")
            else:
                pose = self.moveit2.compute_fk()
                if pose:
                    self.get_logger().info(f"[TUNE] End effector: x={pose.pose.position.x:.4f}, y={pose.pose.position.y:.4f}, z={pose.pose.position.z:.4f}")
            return

        if tune_mode == "hold":
            self.get_logger().info("[TUNE] Closing gripper.")
            self.gripper_control("close")
            return

        if tune_mode == "open":
            self.get_logger().info("[TUNE] Opening gripper.")
            self.gripper_control("open")
            return

        center_x, center_y, _ = compute_board_center()
        center_hover_pose = make_pose(center_x, center_y, CENTER_HOVER_Z, *TOP_DOWN)

        if tune_mode == "center":
            self.get_logger().info(f"[TUNE] Moving to center hover at x={center_x:.3f}, y={center_y:.3f}, z={CENTER_HOVER_Z:.3f}")
            if not self.plan_and_execute_pose(center_hover_pose, cartesian=False):
                self.get_logger().warn("[TUNE] Joint space move failed — trying cartesian.")
                if not self.plan_and_execute_pose(center_hover_pose, cartesian=True):
                    self.get_logger().error("[TUNE] Failed to reach center hover.")
                    return
            pose = self.moveit2.compute_fk()
            if pose:
                self.get_logger().info(f"[TUNE] End effector: x={pose.pose.position.x:.4f}, y={pose.pose.position.y:.4f}, z={pose.pose.position.z:.4f}")
            self.get_logger().info("[TUNE] Reached center hover successfully.")
            return

        if tune_mode == "direct":
            tune_square = self.get_parameter("tune_square").get_parameter_value().string_value.lower()
            if not tune_square or tune_square not in SQUARE_COORDS_M:
                self.get_logger().error(f"[TUNE] Invalid or missing tune_square: '{tune_square}'.")
                return
            square_center = square_center_in_world(tune_square)
            hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
            square_hover = make_pose(square_center.position.x, square_center.position.y, hover_z, *TOP_DOWN)
            square_pick = make_pose(square_center.position.x, square_center.position.y, BOARD_Z + PLACE_Z_OFFSET, *TOP_DOWN)

            self.get_logger().info(f"[TUNE] Direct move to {tune_square} hover (no center hover).")
            if not self.plan_and_execute_pose(square_hover, cartesian=False):
                self.get_logger().warn(f"[TUNE] Joint space move to {tune_square} failed — trying cartesian.")
                if not self.plan_and_execute_pose(square_hover, cartesian=True):
                    self.get_logger().error(f"[TUNE] Failed to reach {tune_square} hover.")
                    return

            self.get_logger().info(f"[TUNE] Descending to board level at {tune_square}.")
            if not self.plan_and_execute_pose(square_pick, cartesian=True):
                self.get_logger().error(f"[TUNE] Descent failed at {tune_square}.")
                return
            self.get_logger().info(f"[TUNE] Reached {tune_square}. Stopping.")
            return

        if tune_mode in ("square", "lift"):
            tune_square = self.get_parameter("tune_square").get_parameter_value().string_value.lower()

            if not tune_square or tune_square not in SQUARE_COORDS_M:
                self.get_logger().error(f"[TUNE] Invalid or missing tune_square: '{tune_square}'.")
                return

            square_center = square_center_in_world(tune_square)
            square_hover = make_pose(square_center.position.x, square_center.position.y, CENTER_HOVER_Z, *TOP_DOWN)
            square_pick = make_pose(square_center.position.x, square_center.position.y, BOARD_Z + PLACE_Z_OFFSET, *TOP_DOWN)

            self.get_logger().info(f"[TUNE] Moving to center hover before {tune_square}.")
            if not self.plan_and_execute_pose(center_hover_pose, cartesian=False):
                self.get_logger().warn("[TUNE] Center hover joint move failed — trying cartesian.")
                if not self.plan_and_execute_pose(center_hover_pose, cartesian=True):
                    self.get_logger().error("[TUNE] Failed to reach center hover. Aborting.")
                    return
            time.sleep(0.1)

            self.get_logger().info(f"[TUNE] Moving to {tune_square} hover.")
            if not self.plan_and_execute_pose(square_hover, cartesian=False):
                self.get_logger().warn(f"[TUNE] Joint space move to {tune_square} failed — trying cartesian.")
                if not self.plan_and_execute_pose(square_hover, cartesian=True):
                    self.get_logger().error(f"[TUNE] Failed to reach {tune_square} hover.")
                    return

            time.sleep(0.08)

            if tune_mode == "square":
                self.get_logger().info(f"[TUNE] Descending to board level at {tune_square}.")
                if not self.plan_and_execute_pose(square_pick, cartesian=True):
                    self.get_logger().error("[TUNE] Descent failed.")
                    return
                self.get_logger().info(f"[TUNE] Reached descent position at {tune_square}.")
                return

            if tune_mode == "lift":
                self.get_logger().info(f"[TUNE] Descending to board level at {tune_square}.")
                if not self.plan_and_execute_pose(square_pick, cartesian=True):
                    self.get_logger().error("[TUNE] Descent failed.")
                    return

                time.sleep(0.08)

                self.get_logger().info(f"[TUNE] Lifting back to z={CENTER_HOVER_Z:.3f}.")
                if not self.plan_and_execute_pose(square_hover, cartesian=True):
                    self.get_logger().warn("[TUNE] Cartesian lift failed — trying joint space.")
                    if not self.plan_and_execute_pose(square_hover, cartesian=False):
                        self.get_logger().error("[TUNE] Failed to lift back.")
                        return

                self.get_logger().info(f"[TUNE] Lift complete at {tune_square}.")
                return

        self.get_logger().error(f"[TUNE] Unknown tune_mode '{tune_mode}'. Valid modes: home, hold, open, center, square, lift, direct.")

    # -------------------------
    # Service callback
    # -------------------------

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

    # -------------------------
    # Lifecycle
    # -------------------------

    def destroy_node(self):
        if self.arduino:
            self.arduino.stop_serial_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PickAndPlaceNode()
    except RuntimeError:
        rclpy.shutdown()
        return
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