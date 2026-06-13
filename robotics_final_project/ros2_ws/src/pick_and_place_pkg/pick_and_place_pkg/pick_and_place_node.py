import time
import threading
import queue
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface
from custom_interface.srv import MoveRobot
from pick_and_place_pkg.constants import (
    SQUARE_COORDS_M, BOARD_Z, BOARD_ORIGIN, HOVER_ABOVE_BOARD_M, CENTER_HOVER_ABOVE_BOARD_M,
    CENTER_HOVER_Z, PLACE_Z_OFFSET, TOP_DOWN, CAPTURE_JOINTS,
    make_pose, square_center_in_world, compute_board_center
)
from pick_and_place_pkg.arduino_client import ArduinoSerialClient, SERIAL_AVAILABLE, list_ports

# from std_msgs.msg import String  # Uncomment when button integration is ready


class PickAndPlaceNode(Node):

    # Initializes the node, creates the move_robot service, runs all setup steps, and fails fast if MoveIt is unavailable.
    def __init__(self):
        super().__init__("pick_and_place_node")
        # The move_robot service is NOT created here — it lives on a separate MoveRobotServiceNode with
        # its OWN executor (see main()). Sharing one executor between the move_group action client and
        # the service corrupted the service's wait set after the first motion: an rcl_action crash under
        # a MultiThreadedExecutor, a silent stall under a SingleThreadedExecutor. Isolating them fixes it.
        self.setup_parameters()
        self.setup_moveit()
        self.setup_gripper()
        self.setup_arduino()
        if not self.wait_for_moveit():
            self.get_logger().error("MoveIt not available. Shutting down.")
            raise RuntimeError("MoveIt not available.")
        # self.setup_button_publisher()  # Uncomment when button integration is ready
        # Async motion: MoveRobotServiceNode enqueues a move onto this shared queue and returns ACCEPTED
        # immediately. The motion itself runs on the MAIN thread in run_motion_loop() (see main()), NOT
        # on an executor thread — mirroring snake_test and the tune/test tasks, the only motion paths
        # proven to run many moves without wedging. The move_complete result is published here.
        self.move_complete_pub = self.create_publisher(String, 'move_complete', 10)
        self.motion_queue = queue.Queue()
        self.get_logger().info("PickAndPlaceNode is ready.")

    # -------------------------
    # Setup
    # -------------------------

    # Declares all ROS parameters and the home joint configuration.
    def setup_parameters(self):
        self.declare_parameter("arduino_enabled", True)
        self.declare_parameter("arduino_port", "/dev/ttyUSB0")
        self.declare_parameter("arduino_baud", 115200)
        self.declare_parameter("auto_detect_arduino", True)
        self.declare_parameter("task", "serve")
        self.declare_parameter("tune_mode", "center")
        self.declare_parameter("tune_square", "b3")
        self.declare_parameter("test_square", "e4")
        self.declare_parameter("castling_side", "kingside")
        self.declare_parameter("src", "e2")
        self.declare_parameter("dst", "e4")
        self.j_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Creates the MoveIt2 interface for the arm.
    def setup_moveit(self):
        # Mutually exclusive (was Reentrant): the original reason for a reentrant group — the motion
        # ran inside the service callback and needed MoveIt's callbacks to run concurrently — is gone
        # now that motion runs on the main thread. Serializing the move_group action client's callbacks
        # avoids the rcl_action "wait set index for status subscription is out of bounds" crash that a
        # MultiThreadedExecutor hit while indexing the action client's entities mid-motion.
        self.moveit_callback_group = MutuallyExclusiveCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
            use_move_group_action=True,
            callback_group=self.moveit_callback_group,
        )

    # Creates the gripper interface, leaving it None if initialization fails.
    def setup_gripper(self):
        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.0],
                closed_gripper_joint_positions=[0.8],
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=True,
            )
            self.get_logger().info("GripperInterface initialized.")
        except Exception as e:
            self.gripper = None
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")

    # Connects to the Arduino serial client, auto-detecting the port unless disabled.
    def setup_arduino(self):
        self.arduino = None
        enabled = self.get_parameter("arduino_enabled").get_parameter_value().bool_value
        if not enabled:
            self.get_logger().info("Arduino disabled via parameter.")
            return

        port = self.get_parameter("arduino_port").get_parameter_value().string_value
        baud = self.get_parameter("arduino_baud").get_parameter_value().integer_value

        if SERIAL_AVAILABLE and list_ports is not None:
            try:
                byid_dir = "/dev/serial/by-id"
                if os.path.isdir(byid_dir):
                    entries = sorted(os.listdir(byid_dir))
                    if entries:
                        port = os.path.join(byid_dir, entries[0])
                        self.get_logger().info(f"Auto-detected Arduino by-id: {port}")
                else:
                    for p in list_ports.comports():
                        desc = (p.description or "").lower()
                        if "arduino" in desc or "usb serial" in desc or "ch341" in desc or "ftdi" in desc:
                            port = p.device
                            self.get_logger().info(f"Auto-detected Arduino: {port}")
                            break
            except Exception as e:
                self.get_logger().debug(f"Auto-detect failed: {e}")

        try:
            self.arduino = ArduinoSerialClient(node=self, port=port, baud=baud)
            self.get_logger().info(f"Arduino client enabled on {port} @ {baud}")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize Arduino client: {e}")
            self.arduino = None

    # Polls MoveIt with compute_fk until it responds or the timeout elapses.
    def wait_for_moveit(self, timeout_s: int = 10) -> bool:
        self.get_logger().info(f"Waiting {timeout_s}s for MoveIt to be ready...")
        start = time.time()

        while time.time() - start < timeout_s:
            try:
                if self.moveit2.compute_fk() is not None:
                    self.get_logger().info("MoveIt is ready.")
                    return True
            except Exception:
                pass
            time.sleep(0.3)

        self.get_logger().warn("Timed out waiting for MoveIt.")
        return False

    # -------------------------
    # Motion primitives
    # -------------------------

    # Moves the arm to a joint configuration and waits for it to finish.
    def move_to_joints(self, joints) -> bool:
        try:
            self.get_logger().info(f"Moving to joints: {[f'{p:.3f}' for p in joints]}")
            self.moveit2.move_to_configuration(joint_positions=joints)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"move_to_configuration failed: {e}")
            return False

    # Plans a path to a pose (joint or cartesian) and executes it, returning success.
    def plan_and_execute_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        try:
            self.get_logger().info(f"[PLAN] Calling moveit2.plan() cartesian={cartesian} target=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
            traj = self.moveit2.plan(
                pose=pose,
                cartesian=cartesian,
                max_step=0.005,
                cartesian_fraction_threshold=0.90 if cartesian else None,
            )
            self.get_logger().info(f"[PLAN] moveit2.plan() returned — traj is None: {traj is None}")
            if traj is None:
                self.get_logger().error(
                    f"Planning failed (cartesian={cartesian}). "
                    f"Target: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}"
                )
                return False
            self.get_logger().info("[EXEC] Calling moveit2.execute()")
            self.moveit2.execute(traj)
            self.get_logger().info("[EXEC] moveit2.execute() returned")
            self.get_logger().info("[WAIT] Calling moveit2.wait_until_executed()")
            self.moveit2.wait_until_executed()
            self.get_logger().info("[WAIT] moveit2.wait_until_executed() returned")
            return True
        except Exception as e:
            self.get_logger().error(f"plan_and_execute_pose failed: {e}")
            return False

    # -------------------------
    # Gripper
    # -------------------------

    # Opens or closes the gripper, guarding against invalid commands or a missing interface.
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

    # Turns the suction solenoid on or off via the Arduino, with a settle delay after turning off.
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

    # Turns the motor on or off via the Arduino, with a settle delay after turning off.
    def motor_control(self, command: str):
        if command not in ("motor on", "motor off"):
            self.get_logger().error(f"Invalid motor command: '{command}'. Use 'motor on' or 'motor off'.")
            return
        if self.arduino is None:
            self.get_logger().warn("Motor command skipped — no Arduino client available.")
            return
        try:
            ok = self.arduino.send_command(command)
            if ok:
                self.get_logger().info(f"Motor: {command}")
            else:
                self.get_logger().warn(f"Motor command failed: {command}")
        except Exception as e:
            self.get_logger().warn(f"Motor control error: {e}")

        if command == "motor off":
            time.sleep(0.2)

    # -------------------------
    # High level motion
    # -------------------------

    # Builds the top-down hover pose above the center of the board.
    def get_center_hover_pose(self) -> Pose:
        cx, cy, cz = compute_board_center()
        center_hover_z = cz + CENTER_HOVER_ABOVE_BOARD_M
        return make_pose(cx, cy, center_hover_z, *TOP_DOWN)

    # Moves the arm to the center hover pose, falling back from joint to cartesian planning.
    def move_to_center(self) -> bool:
        center_hover_pose = self.get_center_hover_pose()
        self.get_logger().info("[MOVE] Moving to center pose.")
        if not self.plan_and_execute_pose(center_hover_pose, cartesian=False):
            self.get_logger().warn("[MOVE] Joint space to center failed — trying cartesian.")
            if not self.plan_and_execute_pose(center_hover_pose, cartesian=True):
                self.get_logger().error("[MOVE] Failed to reach center pose.")
                return False
        return True

    # Picks up the piece on a square and drops it into the capture basket.
    def capture_piece(self, square: str) -> bool:
        square = square.lower()
        if square not in SQUARE_COORDS_M:
            self.get_logger().error(f"Invalid square passed to capture_piece: {square}")
            return False

        s_center = square_center_in_world(square)
        hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
        board_z = BOARD_Z + PLACE_Z_OFFSET

        square_hover = make_pose(s_center.position.x, s_center.position.y, hover_z, *TOP_DOWN)
        square_pick = make_pose(s_center.position.x, s_center.position.y, board_z, *TOP_DOWN)

        # Step 1 — center pose
        if not self.move_to_center():
            return False

        # Motor on — before moving to the source hover position
        self.motor_control("motor on")

        # Step 2 — hover above captured piece
        self.get_logger().info(f"[CAPTURE] Hovering above {square}.")
        if not self.plan_and_execute_pose(square_hover, cartesian=False):
            self.get_logger().warn("[CAPTURE] Joint space hover failed — trying cartesian.")
            if not self.plan_and_execute_pose(square_hover, cartesian=True):
                self.get_logger().error(f"[CAPTURE] Failed to hover above {square}.")
                return False
        time.sleep(0.08)

        # Step 3 — descend to captured piece
        self.get_logger().info(f"[CAPTURE] Descending to {square}.")
        if not self.plan_and_execute_pose(square_pick, cartesian=True):
            self.get_logger().error(f"[CAPTURE] Failed to descend to {square}.")
            return False
        time.sleep(0.08)

        # Step 4 — grab piece
        self.solenoid_control("sol on")

        # Step 5 — lift to hover
        self.get_logger().info(f"[CAPTURE] Lifting from {square}.")
        if not self.plan_and_execute_pose(square_hover, cartesian=True):
            self.get_logger().warn("[CAPTURE] Cartesian lift failed — trying joint space.")
            if not self.plan_and_execute_pose(square_hover, cartesian=False):
                self.get_logger().error("[CAPTURE] Failed to lift after capture.")
                return False
        time.sleep(0.08)

        # Step 6 — center pose
        if not self.move_to_center():
            return False

        # Step 7 — move to drop position
        self.get_logger().info("[CAPTURE] Moving to drop position.")
        if not self.move_to_joints(CAPTURE_JOINTS):
            self.get_logger().error("[CAPTURE] Failed to reach drop position.")
            return False

        # Step 8 — release piece into basket
        self.solenoid_control("sol off")
        self.get_logger().info("[CAPTURE] Piece dropped into basket.")

        # Motor off — after the piece has been released
        self.motor_control("motor off")

        # Step 9 — return to center pose
        if not self.move_to_center():
            return False

        return True

    # Picks up the piece on the source square and places it on the destination square.
    def move_piece(self, src: str, dst: str) -> bool:
        src = src.lower()
        dst = dst.lower()

        if src not in SQUARE_COORDS_M or dst not in SQUARE_COORDS_M:
            self.get_logger().error(f"Invalid square passed to move_piece: {src} -> {dst}")
            return False

        source_center = square_center_in_world(src)
        target_center = square_center_in_world(dst)

        hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
        board_z = BOARD_Z + PLACE_Z_OFFSET

        source_hover = make_pose(source_center.position.x, source_center.position.y, hover_z, *TOP_DOWN)
        source_pick = make_pose(source_center.position.x, source_center.position.y, board_z, *TOP_DOWN)
        target_hover = make_pose(target_center.position.x, target_center.position.y, hover_z, *TOP_DOWN)
        target_place = make_pose(target_center.position.x, target_center.position.y, board_z, *TOP_DOWN)

        # Step 1 — center pose
        if not self.move_to_center():
            return False

        # Motor on — before moving to the source hover position
        self.motor_control("motor on")

        # Step 2 — hover above source
        self.get_logger().info(f"[MOVE] Hovering above {src}.")
        if not self.plan_and_execute_pose(source_hover, cartesian=False):
            self.get_logger().warn("[MOVE] Joint space hover failed — trying cartesian.")
            if not self.plan_and_execute_pose(source_hover, cartesian=True):
                self.get_logger().error("[MOVE] Failed to hover above source.")
                return False
        time.sleep(0.08)

        # Step 3 — descend to source
        self.get_logger().info(f"[MOVE] Descending to {src}.")
        if not self.plan_and_execute_pose(source_pick, cartesian=True):
            self.get_logger().error("[MOVE] Descent to source failed.")
            return False
        time.sleep(0.08)

        # Step 4 — grab piece
        self.solenoid_control("sol on")

        # Step 5 — lift back to hover
        self.get_logger().info(f"[MOVE] Lifting from {src}.")
        if not self.plan_and_execute_pose(source_hover, cartesian=True):
            self.get_logger().warn("[MOVE] Cartesian lift failed — trying joint space.")
            if not self.plan_and_execute_pose(source_hover, cartesian=False):
                self.get_logger().error("[MOVE] Failed to lift after pick.")
                return False
        time.sleep(0.08)

        # Step 6 — travel to target hover
        self.get_logger().info(f"[MOVE] Travelling to hover above {dst}.")
        dist_xy = math.hypot(
            source_hover.position.x - target_hover.position.x,
            source_hover.position.y - target_hover.position.y
        )
        prefer_cartesian = dist_xy < 0.08

        if prefer_cartesian:
            if not self.plan_and_execute_pose(target_hover, cartesian=True):
                self.get_logger().warn("[MOVE] Cartesian travel failed — trying joint space.")
                if not self.plan_and_execute_pose(target_hover, cartesian=False):
                    self.get_logger().error("[MOVE] Failed to travel to target hover.")
                    return False
        else:
            if not self.plan_and_execute_pose(target_hover, cartesian=False):
                self.get_logger().warn("[MOVE] Joint space travel failed — trying cartesian.")
                if not self.plan_and_execute_pose(target_hover, cartesian=True):
                    self.get_logger().error("[MOVE] Failed to travel to target hover.")
                    return False
        time.sleep(0.08)

        # Step 7 — descend to target
        self.get_logger().info(f"[MOVE] Descending to {dst}.")
        if not self.plan_and_execute_pose(target_place, cartesian=True):
            self.get_logger().warn("[MOVE] Direct descent failed — trying staged descent.")
            ok_place = True
            for i in range(1, 4):
                z_step = target_hover.position.z + (target_place.position.z - target_hover.position.z) * (i / 3)
                step_pose = make_pose(target_place.position.x, target_place.position.y, z_step, *TOP_DOWN)
                if not self.plan_and_execute_pose(step_pose, cartesian=True):
                    ok_place = False
                    break
                time.sleep(0.05)
            if not ok_place:
                self.get_logger().error("[MOVE] Staged descent to target failed.")
                return False

        # Step 8 — release piece
        self.solenoid_control("sol off")

        # Motor off — after the piece has been released
        self.motor_control("motor off")

        # Step 9 — retract to hover
        self.get_logger().info(f"[MOVE] Retracting from {dst}.")
        if not self.plan_and_execute_pose(target_hover, cartesian=True):
            self.get_logger().warn("[MOVE] Cartesian retract failed — trying joint space.")
            self.plan_and_execute_pose(target_hover, cartesian=False)

        # Step 10 — center pose
        self.move_to_center()

        return True

    # -------------------------
    # Tasks
    # -------------------------

    # Runs the requested tune mode for manual calibration and testing of arm poses.
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

        # Tune mode "origin" — hover above the board origin, then descend to board level so the position can be physically verified.
        if tune_mode == "origin":
            origin_x, origin_y, _ = BOARD_ORIGIN
            origin_hover_pose = make_pose(origin_x, origin_y, CENTER_HOVER_Z, *TOP_DOWN)
            origin_board_pose = make_pose(origin_x, origin_y, BOARD_Z + PLACE_Z_OFFSET, *TOP_DOWN)

            self.get_logger().info(f"[TUNE] Moving to origin hover at x={origin_x:.4f}, y={origin_y:.4f}, z={CENTER_HOVER_Z:.4f}")
            if not self.plan_and_execute_pose(origin_hover_pose, cartesian=False):
                self.get_logger().warn("[TUNE] Joint space move failed — trying cartesian.")
                if not self.plan_and_execute_pose(origin_hover_pose, cartesian=True):
                    self.get_logger().error("[TUNE] Failed to reach origin hover.")
                    return

            self.get_logger().info(f"[TUNE] Descending to board origin at x={origin_x:.4f}, y={origin_y:.4f}, z={BOARD_Z + PLACE_Z_OFFSET:.4f}")
            if not self.plan_and_execute_pose(origin_board_pose, cartesian=True):
                self.get_logger().error("[TUNE] Descent to origin failed.")
                return

            self.get_logger().info("[TUNE] Reached board origin. Stopping for verification.")
            return

        if tune_mode in ("square", "lift", "direct"):
            tune_square = self.get_parameter("tune_square").get_parameter_value().string_value.lower()
            if not tune_square or tune_square not in SQUARE_COORDS_M:
                self.get_logger().error(f"[TUNE] Invalid or missing tune_square: '{tune_square}'.")
                return

            square_center = square_center_in_world(tune_square)
            hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
            square_hover = make_pose(square_center.position.x, square_center.position.y, hover_z, *TOP_DOWN)
            square_pick = make_pose(square_center.position.x, square_center.position.y, BOARD_Z + PLACE_Z_OFFSET, *TOP_DOWN)

            if tune_mode == "direct":
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
                self.get_logger().info(f"[TUNE] Lifting back to hover.")
                if not self.plan_and_execute_pose(square_hover, cartesian=True):
                    self.get_logger().warn("[TUNE] Cartesian lift failed — trying joint space.")
                    if not self.plan_and_execute_pose(square_hover, cartesian=False):
                        self.get_logger().error("[TUNE] Failed to lift back.")
                        return
                self.get_logger().info(f"[TUNE] Lift complete at {tune_square}.")
                return

        self.get_logger().error(f"[TUNE] Unknown tune_mode '{tune_mode}'. Valid modes: home, hold, open, center, origin, square, lift, direct.")

    # Tests the full capture sequence on the configured test_square.
    def task_test_capture(self):
        test_square = self.get_parameter("test_square").get_parameter_value().string_value.lower()
        self.get_logger().info(f"[TEST CAPTURE] Testing capture sequence on {test_square}.")
        if self.capture_piece(test_square):
            self.get_logger().info(f"[TEST CAPTURE] Capture sequence on {test_square} succeeded.")
        else:
            self.get_logger().error(f"[TEST CAPTURE] Capture sequence on {test_square} failed.")

    # Tests the full castling sequence (king first, then rook) for the configured castling_side.
    def task_test_castling(self):
        castling_side = self.get_parameter("castling_side").get_parameter_value().string_value.lower()

        if castling_side == "kingside":
            king_src, king_dst = "e1", "g1"
            rook_src, rook_dst = "h1", "f1"
        elif castling_side == "queenside":
            king_src, king_dst = "e1", "c1"
            rook_src, rook_dst = "a1", "d1"
        else:
            self.get_logger().error(f"[TEST CASTLING] Invalid castling_side: '{castling_side}'. Use 'kingside' or 'queenside'.")
            return

        self.get_logger().info(f"[TEST CASTLING] Testing {castling_side} castling.")

        self.get_logger().info(f"[TEST CASTLING] Moving king {king_src} -> {king_dst}.")
        if not self.move_piece(king_src, king_dst):
            self.get_logger().error(f"[TEST CASTLING] King move {king_src} -> {king_dst} failed.")
            return

        self.get_logger().info(f"[TEST CASTLING] Moving rook {rook_src} -> {rook_dst}.")
        if not self.move_piece(rook_src, rook_dst):
            self.get_logger().error(f"[TEST CASTLING] Rook move {rook_src} -> {rook_dst} failed.")
            return

        self.get_logger().info(f"[TEST CASTLING] {castling_side} castling sequence complete.")

    # Tests the full move_piece sequence from the configured src square to the dst square.
    def task_test_move(self):
        src = self.get_parameter("src").get_parameter_value().string_value.lower()
        dst = self.get_parameter("dst").get_parameter_value().string_value.lower()

        if src not in SQUARE_COORDS_M or dst not in SQUARE_COORDS_M:
            self.get_logger().error(f"[TEST MOVE] Invalid square(s): {src} -> {dst}.")
            return

        self.get_logger().info(f"[TEST MOVE] Testing move sequence {src} -> {dst}.")
        if self.move_piece(src, dst):
            self.get_logger().info(f"[TEST MOVE] Move sequence {src} -> {dst} succeeded.")
        else:
            self.get_logger().error(f"[TEST MOVE] Move sequence {src} -> {dst} failed.")

    # -------------------------
    # Service callback
    # -------------------------

    # Returns True if the move is a castling move (king moves two squares horizontally from its home square).
    def is_castling_move(self, src, dst):
        return (
            src in ("e1", "e8")
            and dst in ("g1", "c1", "g8", "c8")
            and src[1] == dst[1]
            and abs(ord(src[0]) - ord(dst[0])) == 2
        )

    # Returns the (rook_src, rook_dst) squares for the castling identified by the king's destination square.
    def get_castling_rook_move(self, src, dst):
        rook_moves = {
            "g1": ("h1", "f1"),  # white kingside
            "c1": ("a1", "d1"),  # white queenside
            "g8": ("h8", "f8"),  # black kingside
            "c8": ("a8", "d8"),  # black queenside
        }
        return rook_moves[dst]

    # Main-thread loop (serve mode): blocks pulling queued moves and runs each to completion (capture,
    # castling, normal), then publishes the result on 'move_complete'. Runs on the MAIN thread — the
    # same pattern snake_test and the tune/test tasks use — so MoveIt's long wait_until_executed() is
    # never driven from an executor thread, which is what wedged the executor after one move.
    def run_motion_loop(self):
        while rclpy.ok():
            try:
                src, dst, is_capture = self.motion_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                success = True
                if is_capture:
                    self.get_logger().info(f"Capture move — removing piece from {dst} first.")
                    success = self.capture_piece(dst)
                if success:
                    if self.is_castling_move(src, dst):
                        self.get_logger().info(f"Castling move detected: {src}{dst}.")
                        success = self.move_piece(src, dst)
                        if success:
                            rook_src, rook_dst = self.get_castling_rook_move(src, dst)
                            self.get_logger().info(f"Moving castling rook: {rook_src} -> {rook_dst}.")
                            success = self.move_piece(rook_src, rook_dst)
                    else:
                        success = self.move_piece(src, dst)
            except Exception as e:
                self.get_logger().error(f"Motion loop exception: {e}")
                success = False
            status = "SUCCESS" if success else "ERROR"
            msg = String()
            msg.data = status
            self.move_complete_pub.publish(msg)
            self.get_logger().info(f"[WORKER] Move complete — published status: {status}")

    # -------------------------
    # Lifecycle
    # -------------------------

    # Closes the Arduino connection and shuts the node down.
    def destroy_node(self):
        if self.arduino:
            self.arduino.stop_serial_connection()
        super().destroy_node()


# Isolated node that owns ONLY the move_robot service. It runs on its own executor (see main()), so it
# is completely separate from the MoveIt action client's wait set — the action client can crash or stall
# its own executor without ever stopping this service from accepting the next move. The handler just
# validates the UCI and enqueues the move onto the shared queue; the main thread runs the motion.
class MoveRobotServiceNode(Node):

    def __init__(self, motion_queue):
        super().__init__("move_robot_service_node")
        self.motion_queue = motion_queue
        self.move_service = self.create_service(MoveRobot, "move_robot", self.handle_move_robot)
        self.get_logger().info("MoveRobotServiceNode is ready.")

    # Validates the UCI, enqueues the move for the main-thread motion loop, and returns ACCEPTED at once.
    def handle_move_robot(self, request, response):
        uci = request.best_uci.strip().lower()
        is_capture = request.is_capture
        self.get_logger().info(f"[SERVICE] Received move request: {uci} (capture={is_capture})")

        if len(uci) != 4:
            response.robot_status_message = "ERROR: Invalid UCI"
            return response

        self.motion_queue.put((uci[:2], uci[2:], is_capture))
        response.robot_status_message = "ACCEPTED"
        self.get_logger().info("[SERVICE] Move queued — returning ACCEPTED")
        return response


# Initializes ROS, starts the nodes and executors, runs the requested task, then shuts down.
def main(args=None):
    rclpy.init(args=args)
    try:
        node = PickAndPlaceNode()
    except RuntimeError:
        rclpy.shutdown()
        return
    # Two nodes, two executors, two independent wait sets. The MoveIt node (action client + motion) is
    # spun by its own executor — exactly the snake_test pattern, proven to run many moves with the
    # motion on the main thread. The move_robot service lives on a SEPARATE node with its OWN executor,
    # so the action client's wait-set problems can never stop the service from accepting the next move.
    service_node = MoveRobotServiceNode(node.motion_queue)

    moveit_executor = SingleThreadedExecutor()
    moveit_executor.add_node(node)
    service_executor = SingleThreadedExecutor()
    service_executor.add_node(service_node)

    # Spin each executor in its own background thread; log any crash (daemon threads die silently).
    def spin_executor(executor, label):
        try:
            executor.spin()
        except Exception as e:
            node.get_logger().error(f"[EXECUTOR:{label}] spin() crashed: {e}")

    threading.Thread(target=spin_executor, args=(moveit_executor, "moveit"), daemon=True).start()
    threading.Thread(target=spin_executor, args=(service_executor, "service"), daemon=True).start()

    task = node.get_parameter("task").get_parameter_value().string_value

    try:
        if task == "tune":
            node.task_tune()
        elif task == "test_capture":
            node.task_test_capture()
        elif task == "test_castling":
            node.task_test_castling()
        elif task == "test_move":
            node.task_test_move()
        elif task == "home":
            node.move_to_joints(node.j_home)
        else:
            node.get_logger().info("PickAndPlaceNode serving move_robot requests.")
            node.run_motion_loop()   # motion runs HERE on the main thread; executor spins in background
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino:
            node.arduino.stop_serial_connection()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
