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
    SQUARE_COORDS_M, BOARD_Z, HOVER_ABOVE_BOARD_M,
    PLACE_Z_OFFSET, TOP_DOWN, SOLENOID_OFF_DELAY_DEFAULT,
    make_pose, square_center_in_world
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
            from pick_and_place_pkg.tune import task_tune
            task_tune(node)
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