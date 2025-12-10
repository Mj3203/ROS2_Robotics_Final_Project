#!/usr/bin/env python3
"""
Complete PickAndPlaceNode with Arduino-controlled solenoid, auto-detect,
arc motion leg, full tune/move implementations, AND new button-publishing
support for GameOperationNode.

Key points UPDATED:
- Node renamed ChessMoverNode -> PickAndPlaceNode
- ArduinoSerialClient now detects button messages
- PickAndPlaceNode now publishes /button_feed
"""

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
from std_msgs.msg import String   # NEW IMPORT FOR BUTTON TOPIC

# Try to import pyserial (non-fatal)
try:
    import serial
    from serial.tools import list_ports
    SERIAL_AVAILABLE = True
except Exception:
    serial = None
    list_ports = None
    SERIAL_AVAILABLE = False


# -------------------------
# Tunable defaults
# -------------------------
SOLENOID_OFF_DELAY_DEFAULT = 0.2


# -------------------------
# Units and board constants
# -------------------------
INCH = 0.0254
HOVER_ABOVE_BOARD_M = 2.0 * INCH
CENTER_HOVER_ABOVE_BOARD_M = 8.0 * INCH
PLACE_Z_OFFSET = 0.0
BOARD_Z = 8.4 * INCH
BOARD_ORIGIN = (-6.55*INCH, 7.4*INCH, BOARD_Z)
TOP_DOWN = (-0.7071, 0.7071, 0.0, 0.0)

SQUARE_COORDS_M = {
    'a1': (0.020637, 0.020637), 'a2': (0.020637, 0.061912), 'a3': (0.020637, 0.103188), 'a4': (0.020637, 0.144462),
    'a5': (0.020637, 0.185737), 'a6': (0.020637, 0.227013), 'a7': (0.020637, 0.268287), 'a8': (0.020637, 0.309563),
    'b1': (0.061912, 0.020637), 'b2': (0.061912, 0.061912), 'b3': (0.061912, 0.103188), 'b4': (0.061912, 0.144462),
    'b5': (0.061912, 0.185737), 'b6': (0.061912, 0.227013), 'b7': (0.061912, 0.268287), 'b8': (0.061912, 0.309563),
    'c1': (0.103188, 0.020637), 'c2': (0.103188, 0.061912), 'c3': (0.103188, 0.103188), 'c4': (0.103188, 0.144462),
    'c5': (0.103188, 0.185737), 'c6': (0.103188, 0.227013), 'c7': (0.103188, 0.268287), 'c8': (0.103188, 0.309563),
    'd1': (0.144462, 0.020637), 'd2': (0.144462, 0.061912), 'd3': (0.144462, 0.103188), 'd4': (0.144462, 0.144462),
    'd5': (0.144462, 0.185737), 'd6': (0.144462, 0.227013), 'd7': (0.144462, 0.268287), 'd8': (0.144462, 0.309563),
    'e1': (0.185737, 0.020637), 'e2': (0.185737, 0.061912), 'e3': (0.185737, 0.103188), 'e4': (0.185737, 0.144462),
    'e5': (0.185737, 0.185737), 'e6': (0.185737, 0.227013), 'e7': (0.185737, 0.268287), 'e8': (0.185737, 0.309563),
    'f1': (0.227013, 0.020637), 'f2': (0.227013, 0.061912), 'f3': (0.227013, 0.103188), 'f4': (0.227013, 0.144462),
    'f5': (0.227013, 0.185737), 'f6': (0.227013, 0.227013), 'f7': (0.227013, 0.268287), 'f8': (0.227013, 0.309563),
    'g1': (0.268287, 0.020637), 'g2': (0.268287, 0.061912), 'g3': (0.268287, 0.103188), 'g4': (0.268287, 0.144462),
    'g5': (0.268287, 0.185737), 'g6': (0.268287, 0.227013), 'g7': (0.268287, 0.268287), 'g8': (0.268287, 0.309563),
    'h1': (0.309563, 0.020637), 'h2': (0.309563, 0.061912), 'h3': (0.309563, 0.103188), 'h4': (0.309563, 0.144462),
    'h5': (0.309563, 0.185737), 'h6': (0.309563, 0.227013), 'h7': (0.309563, 0.268287), 'h8': (0.309563, 0.309563)
}

# -------------------------
# Helper Functions
# -------------------------
def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p


def square_center_in_world(square: str):
    sq = square.upper()
    if sq not in SQUARE_COORDS_M:
        raise ValueError(f"Unknown square {sq}")
    x_rel, y_rel = SQUARE_COORDS_M[sq]
    bx, by, bz = BOARD_ORIGIN
    return make_pose(bx + x_rel, by + y_rel, bz, *TOP_DOWN)


def compute_board_center():
    xs = [v[0] for v in SQUARE_COORDS_M.values()]
    ys = [v[1] for v in SQUARE_COORDS_M.values()]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    center_rel_x = (min_x + max_x) / 2.0
    center_rel_y = (min_y + max_y) / 2.0
    bx, by, bz = BOARD_ORIGIN
    return (bx + center_rel_x, by + center_rel_y, bz)

# -------------------------
# Arduino Serial Client
# -------------------------
class ArduinoSerialClient:
    """
    Background reader + send() method + NEW button detection.
    """

    # =======================================================
    # CHANGE: Added button_callback to constructor
    # =======================================================
    def __init__(self, node: Node, port="/dev/ttyUSB0",
                 baud=115200, timeout=1.0,
                 button_callback=None):

        self.node = node
        self.port = port
        self.baud = int(baud)
        self.timeout = float(timeout)
        self.button_callback = button_callback   # NEW
        self._ser = None
        self._reader_thread = None
        self._running = False
        self._lock = threading.Lock()

        if not SERIAL_AVAILABLE:
            self.node.get_logger().warn("pyserial not available. Arduino disabled.")
            return

        self._start()

    def _start(self):
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._run_reader, daemon=True
        )
        self._reader_thread.start()

    def _open_serial(self):
        try:
            s = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.2)
            self.node.get_logger().info(
                f"ArduinoSerialClient: opened {self.port} @ {self.baud}"
            )
            return s
        except Exception as e:
            self.node.get_logger().warn(f"ArduinoSerialClient: cannot open {self.port}: {e}")
            return None

    def _run_reader(self):
        reconnect_delay = 2.0

        while self._running:
            if self._ser is None:
                self._ser = self._open_serial()
                if self._ser is None:
                    time.sleep(reconnect_delay)
                    continue

            try:
                if self._ser.in_waiting:
                    raw = self._ser.readline().decode(errors='ignore').strip()
                    if raw:
                        self.node.get_logger().debug(f"[Arduino] {raw}")

                        # =======================================================
                        # NEW: Button detection logic
                        # Arduino must send "BUTTON" or "EVENT:BUTTON"
                        # =======================================================
                        if raw.upper().startswith("BUTTON") or "EVENT:BUTTON" in raw.upper():
                            self.node.get_logger().info(f"Arduino button detected: {raw}")
                            if self.button_callback:
                                self.button_callback()

                else:
                    time.sleep(0.05)

            except Exception as e:
                self.node.get_logger().warn(f"ArduinoSerialClient read error: {e}")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(reconnect_delay)

    def send(self, line: str):
        if not SERIAL_AVAILABLE:
            return False

        with self._lock:
            if self._ser is None:
                self._ser = self._open_serial()
                if self._ser is None:
                    return False

            try:
                msg = (line.strip() + '\n').encode()
                self._ser.write(msg)
                self._ser.flush()
                return True
            except Exception:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                return False

    def stop(self):
        self._running = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
# -------------------------
# Main node
# -------------------------
class PickAndPlaceNode(Node):   # UPDATED NAME
    def __init__(self):
        super().__init__("pick_and_place_node")

        # =======================================================
        # NEW: Publisher for button → GameOperation
        # =======================================================
        self.button_pub = self.create_publisher(
            String,
            "button_feed",
            10
        )
        self.get_logger().info("button_feed publisher created.")

        # basic params
        self.declare_parameter("task", "move_B3_B5")
        self.declare_parameter("cartesian_fraction_threshold", 0.90)
        self.declare_parameter("tune_mode", "center")
        self.declare_parameter("tune_square", "B3")
        self.declare_parameter("src", "B3")
        self.declare_parameter("dst", "B5")

        # Arduino params & auto-detect
        self.declare_parameter("arduino_enabled", True)
        self.declare_parameter("arduino_port", "/dev/ttyUSB0")
        self.declare_parameter("arduino_baud", 115200)
        self.declare_parameter("solenoid_off_delay", float(SOLENOID_OFF_DELAY_DEFAULT))
        self.declare_parameter("auto_detect_arduino", True)

        # MoveIt2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint_1", "joint_2", "joint_3",
                "joint_4", "joint_5", "joint_6"
            ],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        # gripper
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

        self.gripper_closed = False
        self.gripper_locked = False
        self.j_home = [0.0] * 6

        # =======================================================
        # Arduino initialization (with button handler)
        # =======================================================
        self.arduino = None
        try:
            enabled = self.get_parameter("arduino_enabled").get_parameter_value().bool_value
            port_param = self.get_parameter("arduino_port").get_parameter_value().string_value
            baud = self.get_parameter("arduino_baud").get_parameter_value().integer_value
            autod = self.get_parameter("auto_detect_arduino").get_parameter_value().bool_value

            chosen_port = port_param

            if enabled and autod and SERIAL_AVAILABLE and list_ports is not None:
                try:
                    # check /dev/serial/by-id first
                    byid_dir = "/dev/serial/by-id"
                    if os.path.isdir(byid_dir):
                        entries = sorted(os.listdir(byid_dir))
                        if entries:
                            chosen_port = os.path.join(byid_dir, entries[0])
                            self.get_logger().info(f"Auto-detected Arduino by-id: {chosen_port}")

                    # if still default, try comports
                    if chosen_port == port_param:
                        for p in list_ports.comports():
                            desc = (p.description or "").lower()
                            if ("arduino" in desc
                                or "usb serial" in desc
                                or "ch341" in desc
                                or "ftdi" in desc):
                                chosen_port = p.device
                                self.get_logger().info(f"Auto-detected Arduino: {chosen_port}")
                                break

                except Exception as e:
                    self.get_logger().debug(f"Auto-detect failed: {e}")

            if enabled:
                # =======================================================
                # PASS THE CALLBACK TO THE ARDUINO CLIENT
                # =======================================================
                self.arduino = ArduinoSerialClient(
                    node=self,
                    port=chosen_port,
                    baud=baud,
                    button_callback=self._on_button_pressed  # NEW
                )
                self.get_logger().info(f"Arduino client enabled on {chosen_port}@{baud}")
            else:
                self.get_logger().info("Arduino disabled via parameter.")

        except Exception as e:
            self.get_logger().warn(f"Failed to initialize Arduino client: {e}")
            self.arduino = None

        time.sleep(0.1)

        from custom_interface.srv import MoveRobot
        
        self.move_service = self.create_service(
            MoveRobot,
            "move_robot",
            self._handle_move_robot
        )

        self.get_logger().info("PickAndPlaceNode: move_robot service READY.")

    # =======================================================
    # NEW ----------------------------------------------------
    # Button handler → publishes to GameOperation
    # =======================================================
    def _on_button_pressed(self):
        """
        Called by ArduinoSerialClient whenever the Arduino sends a
        BUTTON event. Publishes to /button_feed.
        """
        try:
            msg = String()
            msg.data = "BUTTON_PRESSED"
            self.button_pub.publish(msg)
            self.get_logger().info("Published button press → /button_feed")
        except Exception as e:
            self.get_logger().error(f"Failed to publish button event: {e}")

    # =======================================================
    # Wait for MoveIt and joint_state availability
    # =======================================================
    def wait_for_moveit_and_state(self, timeout_s: int = 60) -> bool:
        import subprocess
        self.get_logger().info(
            f"Waiting up to {timeout_s}s for planner and /joint_states..."
        )

        start = time.time()
        planner_ready = False

        # Wait for planning service
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

        # Wait for joint_states
        expected = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
        joint_ok = False
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
                all_present = True
                for j in expected:
                    if j not in joint_out:
                        all_present = False
                        break
                if all_present:
                    joint_ok = True
                    break

            time.sleep(0.3)

        if not joint_ok:
            self.get_logger().warn("Timed out waiting for joint_states.")

        ok = planner_ready and joint_ok
        if ok:
            self.get_logger().info("Planner and joint_states detected.")

        return ok

    # =======================================================
    # SIMPLE JOINT MOTION
    # =======================================================
    def move_to_joints(self, joints) -> bool:
        try:
            self.get_logger().info(
                f"Moving to joints: {[f'{p:.3f}' for p in joints]}"
            )
            self.moveit2.move_to_configuration(joint_positions=joints)
            self.moveit2.wait_until_executed()
            return True

        except Exception as e:
            self.get_logger().error(f"move_to_configuration failed: {e}")
            return False

    # =======================================================
    # TRAJECTORY EXECUTION (pose)
    # =======================================================
    def plan_and_execute_pose(self, pose: Pose, cartesian: bool = False) -> bool:

        try:
            threshold = float(
                self.get_parameter("cartesian_fraction_threshold")
                .get_parameter_value().double_value
            )
        except Exception:
            threshold = 0.90

        if threshold <= 0.0 or threshold > 1.0:
            threshold = 0.90

        try:
            traj = self.moveit2.plan(
                pose=pose,
                cartesian=cartesian,
                max_step=0.005,
                cartesian_fraction_threshold=(
                    threshold if cartesian else None
                ),
            )

        except Exception as e:
            self.get_logger().warn(f"Planning exception: {e}")
            return False

        if traj is None:
            self.get_logger().warn(
                f"Planning to pose failed (cart={cartesian}) "
                f"Target: x={pose.position.x:.3f}, "
                f"y={pose.position.y:.3f}, "
                f"z={pose.position.z:.3f}"
            )
            return False

        try:
            self.moveit2.execute(traj)
            self.moveit2.wait_until_executed()
            return True

        except Exception as e:
            self.get_logger().warn(f"Execution failed: {e}")
            return False
    
    # =======================================================
    # GRIPPER CONTROL HELPERS
    # =======================================================
    def lock_gripper(self):
        """Prevents accidental opening during tuning."""
        if self.gripper is None:
            self.get_logger().warn("lock_gripper() requested but gripper is missing.")
            self.gripper_locked = True
            return

        if self.gripper_locked:
            self.get_logger().info("Gripper already locked.")
            return

        # Store original open() if not saved
        try:
            if not hasattr(self.gripper, "_original_open"):
                self.gripper._original_open = getattr(self.gripper, "open", None)
        except Exception:
            self.get_logger().debug("Could not store original gripper.open.")

        # Override open()
        def _blocked_open(*args, **kwargs):
            self.get_logger().warn("Gripper open blocked because gripper is locked.")
            return False

        try:
            setattr(self.gripper, "open", _blocked_open)
            self.gripper_locked = True
            self.get_logger().info("Gripper locked successfully.")
        except Exception as e:
            self.get_logger().warn(f"Failed to lock gripper: {e}")
            self.gripper_locked = True

    def open_gripper_now(self):
        """Explicit command from tuning."""
        if self.gripper_locked:
            self.get_logger().warn("Cannot open: gripper is locked.")
            return False

        if self.gripper is None:
            self.get_logger().warn("Cannot open: gripper interface is missing.")
            return False

        try:
            self.get_logger().info("Opening gripper...")
            self.gripper.open()
            time.sleep(0.15)
            self.gripper_closed = False
            return True
        except Exception as e:
            self.get_logger().warn(f"Gripper open failed: {e}")
            return False

    def close_gripper_now(self):
        """Explicit command from tuning."""
        if self.gripper is None:
            self.get_logger().warn("Cannot close: gripper interface missing.")
            self.gripper_closed = True
            return False

        try:
            self.get_logger().info("Closing gripper...")
            self.gripper.close()
            time.sleep(0.15)
            self.gripper_closed = True
            return True

        except Exception as e:
            self.get_logger().warn(f"Gripper close failed: {e}")
            return False

    # =======================================================
    # MAIN MOVE FUNCTION (pick and place)
    # =======================================================
    def move_piece(self, src: str, dst: str, piece_object_id: str = None) -> bool:

        src = src.lower()
        dst = dst.lower()

        if src not in SQUARE_COORDS_M or dst not in SQUARE_COORDS_M:
            self.get_logger().error("Invalid square passed to move_piece.")
            return False

        s_center = square_center_in_world(src)
        t_center = square_center_in_world(dst)

        hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
        pick_z = BOARD_Z + PLACE_Z_OFFSET
        place_z = BOARD_Z + PLACE_Z_OFFSET

        shover = make_pose(s_center.position.x, s_center.position.y, hover_z, *TOP_DOWN)
        spick = make_pose(s_center.position.x, s_center.position.y, pick_z, *TOP_DOWN)

        thover = make_pose(t_center.position.x, t_center.position.y, hover_z, *TOP_DOWN)
        tplace = make_pose(t_center.position.x, t_center.position.y, place_z, *TOP_DOWN)

        # ---------------------------------------------------
        # MOVE: Hover to source
        # ---------------------------------------------------
        if not self.plan_and_execute_pose(shover, cartesian=False):
            self.get_logger().warn("Hover to source failed — trying offsets.")
            success = False

            for dx, dy in [(0,0),(0.01,0),(-0.01,0),(0,0.01),(0,-0.01)]:
                trial = make_pose(
                    shover.position.x+dx,
                    shover.position.y+dy,
                    shover.position.z,
                    *TOP_DOWN
                )
                if self.plan_and_execute_pose(trial, cartesian=False):
                    success = True
                    break

            if not success:
                self.get_logger().error("Failed to reach source hover.")
                return False

        time.sleep(0.08)

        # ---------------------------------------------------
        # SOLENOID ON
        # ---------------------------------------------------
        try:
            if self.arduino:
                ok = self.arduino.send("sol on")
                if ok:
                    self.get_logger().info("Solenoid ON sent.")
                else:
                    self.get_logger().warn("Solenoid ON failed to send.")
        except Exception as e:
            self.get_logger().warn(f"Solenoid ON error: {e}")

        # ---------------------------------------------------
        # DESCEND TO PICK
        # ---------------------------------------------------
        if not self.plan_and_execute_pose(spick, cartesian=True):
            self.get_logger().warn("Direct descend failed — staging...")
            ok_descend = True

            for i in range(1, 4):
                z_step = shover.position.z + (spick.position.z - shover.position.z) * (i/3)
                step_pose = make_pose(spick.position.x, spick.position.y, z_step, *TOP_DOWN)

                if not self.plan_and_execute_pose(step_pose, cartesian=True):
                    ok_descend = False
                    break

                time.sleep(0.05)

            if not ok_descend:
                self.get_logger().error("Staged descent failed.")
                return False
        # ---------------------------------------------------
        # ATTACH OBJECT (optional, simulation only)
        # ---------------------------------------------------
        if piece_object_id:
            if self.gripper_closed:
                try:
                    self.moveit2.attach_collision_object(
                        piece_object_id,
                        "end_effector_link",
                        touch_links=["right_finger_bottom_joint"]
                    )
                    time.sleep(0.02)
                except Exception as e:
                    self.get_logger().warn(f"Attach object failed: {e}")
            else:
                self.get_logger().warn(
                    "Skipping attach — gripper not closed."
                )

        # ---------------------------------------------------
        # LIFT BACK TO HOVER
        # ---------------------------------------------------
        if not self.plan_and_execute_pose(shover, cartesian=True):
            self.get_logger().warn("Lift failed (cartesian). Trying joint-space.")
            if not self.plan_and_execute_pose(shover, cartesian=False):
                self.get_logger().error("Failed to lift after pick.")
                return False

        time.sleep(0.08)

        # ---------------------------------------------------
        # MOVE TO TARGET HOVER
        # ---------------------------------------------------
        dist_xy = math.hypot(
            shover.position.x - thover.position.x,
            shover.position.y - thover.position.y
        )
        prefer_cart = dist_xy < 0.08

        if prefer_cart:
            if not self.plan_and_execute_pose(thover, cartesian=True):
                self.get_logger().warn("Cartesian travel failed — joint fallback.")
                if not self.plan_and_execute_pose(thover, cartesian=False):
                    self.get_logger().error("Failed travel to target hover.")
                    return False
        else:
            if not self.plan_and_execute_pose(thover, cartesian=False):
                self.get_logger().warn("Joint travel failed — cartesian fallback.")
                if not self.plan_and_execute_pose(thover, cartesian=True):
                    self.get_logger().error("Failed travel to target hover.")
                    return False

        time.sleep(0.08)

        # ---------------------------------------------------
        # DESCEND TO PLACE
        # ---------------------------------------------------
        if not self.plan_and_execute_pose(tplace, cartesian=True):
            self.get_logger().warn("Direct descend to place failed — staged descent.")
            ok_place = True

            for i in range(1, 4):
                z_step = thover.position.z + (tplace.position.z - thover.position.z) * (i/3)
                step_pose = make_pose(tplace.position.x, tplace.position.y, z_step, *TOP_DOWN)

                if not self.plan_and_execute_pose(step_pose, cartesian=True):
                    ok_place = False
                    break

                time.sleep(0.05)

            if not ok_place:
                self.get_logger().error("Failed staged descend to place.")
                return False

        # ---------------------------------------------------
        # DETACH OBJECT (simulation only)
        # ---------------------------------------------------
        if piece_object_id:
            try:
                self.moveit2.detach_collision_object(piece_object_id)
                time.sleep(0.02)
            except Exception as e:
                self.get_logger().warn(f"Detach object failed: {e}")

        # ---------------------------------------------------
        # SOLENOID OFF
        # ---------------------------------------------------
        try:
            if self.arduino:
                ok = self.arduino.send("sol off")
                if ok:
                    self.get_logger().info("Solenoid OFF sent.")
                else:
                    self.get_logger().warn("Solenoid OFF failed to send.")
        except Exception as e:
            self.get_logger().warn(f"Solenoid OFF error: {e}")

        # post-off wait
        try:
            sol_off_delay = float(
                self.get_parameter("solenoid_off_delay")
                .get_parameter_value().double_value
            )
        except Exception:
            sol_off_delay = SOLENOID_OFF_DELAY_DEFAULT

        if sol_off_delay > 0.0:
            self.get_logger().info(
                f"Waiting {sol_off_delay:.3f}s after solenoid off..."
            )
            time.sleep(sol_off_delay)

        # ---------------------------------------------------
        # RETRACT TO TARGET HOVER
        # ---------------------------------------------------
        if not self.plan_and_execute_pose(thover, cartesian=True):
            self.get_logger().warn("Retract failed (cartesian). Trying joint-space.")
            self.plan_and_execute_pose(thover, cartesian=False)

        return True

    # ============================================================
    # NEW — HANDLE MOVE REQUEST FROM GAME_OPERATION
    # ============================================================
    def _handle_move_robot(self, request, response):
        """
        This is called when GameOperation sends a robot move.
        Ex: request.best_uci = 'E2E4'
        """
        uci = request.best_uci.strip().upper()
        self.get_logger().info(f"Received move request from GameOperation: {uci}")

        if len(uci) != 4:
            response.robot_status_message = "ERROR: Invalid UCI"
            return response

        src = uci[:2]
        dst = uci[2:]

        success = self.move_piece(src, dst)

        response.robot_status_message = "SUCCESS" if success else "ERROR"
        return response

    # =======================================================
    # TUNING TASK
    # =======================================================
    def task_tune(self):
        ok = self.wait_for_moveit_and_state(timeout_s=60)
        if not ok:
            self.get_logger().warn("Continuing tuning despite incomplete readiness.")

        tune_mode = (
            self.get_parameter("tune_mode").get_parameter_value().string_value
        )
        tune_mode = tune_mode.lower() if isinstance(tune_mode, str) else "center"

        # ---------------------------------------------------
        # MODE: HOME (go straight up)
        # ---------------------------------------------------
        if tune_mode == "home":
            self.get_logger().info("[TUNE] Moving to HOME pose.")
            if not self.move_to_joints(self.j_home):
                self.get_logger().error("[TUNE] FAILED to move to home.")
            return

        # ---------------------------------------------------
        # MODE: HOLD (lock gripper closed)
        # ---------------------------------------------------
        if tune_mode == "hold":
            self.get_logger().info("[TUNE] HOLD — closing + locking gripper.")

            if not self.close_gripper_now():
                self.get_logger().warn("[TUNE] close_gripper_now() returned False.")

            self.lock_gripper()
            self.get_logger().info("[TUNE] Gripper is now LOCKED.")
            return

        # ---------------------------------------------------
        # MODE: OPEN (force gripper open)
        # ---------------------------------------------------
        if tune_mode == "open":
            self.get_logger().info("[TUNE] OPEN — opening gripper now.")

            ok_open = self.open_gripper_now()
            if ok_open:
                self.get_logger().info("[TUNE] Gripper opened successfully.")
            else:
                self.get_logger().warn("[TUNE] Gripper open failed or was blocked.")
            return

        # ---------------------------------------------------
        # COMPUTE CENTER HOVER (all remaining modes use this)
        # ---------------------------------------------------
        cx, cy, cz = compute_board_center()
        center_hover_z = cz + CENTER_HOVER_ABOVE_BOARD_M
        center_hover_pose = make_pose(cx, cy, center_hover_z, *TOP_DOWN)

        # ---------------------------------------------------
        # MODE: CENTER
        # ---------------------------------------------------
        if tune_mode == "center":
            self.get_logger().info(
                f"[TUNE] CENTER HOVER at "
                f"x={cx:.3f}, y={cy:.3f}, z={center_hover_z:.3f}"
            )

            if not self.plan_and_execute_pose(center_hover_pose, cartesian=False):
                self.get_logger().warn("[TUNE] joint move failed — trying cartesian.")
                if not self.plan_and_execute_pose(center_hover_pose, cartesian=True):
                    self.get_logger().error("[TUNE] FAILED to reach center hover.")
            else:
                self.get_logger().info("[TUNE] Reached center hover successfully.")
            return
        # ---------------------------------------------------
        # MODES: SQUARE and LIFT
        # ---------------------------------------------------
        if tune_mode in ("square", "lift"):
            tune_square = (
                self.get_parameter("tune_square").get_parameter_value().string_value
            )

            if not tune_square:
                self.get_logger().error("[TUNE] Missing required parameter 'tune_square'.")
                return

            tune_square = tune_square.upper()
            if tune_square not in SQUARE_COORDS_M:
                self.get_logger().error(f"[TUNE] Unknown tune_square '{tune_square}'.")
                return

            # => Compute poses
            s_center = square_center_in_world(tune_square)
            hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
            shover = make_pose(
                s_center.position.x, s_center.position.y, hover_z, *TOP_DOWN
            )
            spick = make_pose(
                s_center.position.x, s_center.position.y,
                BOARD_Z + PLACE_Z_OFFSET, *TOP_DOWN
            )

            # ---------------------------------------------------
            # Move to hover
            # ---------------------------------------------------
            self.get_logger().info(
                f"[TUNE] Moving to hover above {tune_square} "
                f"({shover.position.x:.3f}, {shover.position.y:.3f}, {shover.position.z:.3f})"
            )

            if not self.plan_and_execute_pose(shover, cartesian=False):
                self.get_logger().warn(
                    "[TUNE] Hover joint move failed — trying small offsets."
                )
                success = False
                for dx, dy in [(0,0), (0.01,0), (-0.01,0), (0,0.01), (0,-0.01)]:
                    trial = make_pose(
                        shover.position.x + dx,
                        shover.position.y + dy,
                        shover.position.z,
                        *TOP_DOWN
                    )
                    if self.plan_and_execute_pose(trial, cartesian=False):
                        success = True
                        break

                if not success:
                    if not self.plan_and_execute_pose(shover, cartesian=True):
                        self.get_logger().error(
                            "[TUNE] FAILED to reach hover over square."
                        )
                        return

            time.sleep(0.08)

            # ---------------------------------------------------
            # MODE = square → descend and STOP
            # ---------------------------------------------------
            if tune_mode == "square":
                self.get_logger().info(
                    f"[TUNE] Descending to PLACE Z at {tune_square} (no lift after)."
                )

                if not self.plan_and_execute_pose(spick, cartesian=True):
                    self.get_logger().warn(
                        "[TUNE] Direct descend failed — trying staged descent."
                    )
                    steps = 4
                    ok_desc = True
                    for i in range(1, steps+1):
                        z_step = shover.position.z + \
                                 (spick.position.z - shover.position.z) * (i/steps)
                        step_pose = make_pose(
                            spick.position.x, spick.position.y, z_step, *TOP_DOWN
                        )
                        if not self.plan_and_execute_pose(step_pose, cartesian=True):
                            ok_desc = False
                            break
                        time.sleep(0.05)

                    if not ok_desc:
                        self.get_logger().error(
                            "[TUNE] FAILED staged descent for tune_mode='square'."
                        )
                        return

                self.get_logger().info(
                    f"[TUNE] Reached place Z at {tune_square}. Stopping here."
                )
                return

            # ---------------------------------------------------
            # MODE = lift → descend then LIFT back up to hover
            # ---------------------------------------------------
            if tune_mode == "lift":
                self.get_logger().info(
                    f"[TUNE-LIFT] Descending to pick/place Z at {tune_square}."
                )

                if not self.plan_and_execute_pose(spick, cartesian=True):
                    self.get_logger().warn(
                        "[TUNE-LIFT] Direct descend failed — trying staged."
                    )
                    steps = 3
                    ok_desc = True
                    for i in range(1, steps+1):
                        z_step = shover.position.z + \
                                 (spick.position.z - shover.position.z) * (i/steps)
                        step_pose = make_pose(
                            spick.position.x, spick.position.y, z_step, *TOP_DOWN
                        )
                        if not self.plan_and_execute_pose(step_pose, cartesian=True):
                            ok_desc = False
                            break
                        time.sleep(0.05)

                    if not ok_desc:
                        self.get_logger().error(
                            "[TUNE-LIFT] FAILED staged descend."
                        )
                        return

                time.sleep(0.08)

                # LIFT BACK UP
                self.get_logger().info(
                    f"[TUNE-LIFT] Lifting back to hover (z={hover_z:.3f})."
                )

                if not self.plan_and_execute_pose(shover, cartesian=True):
                    self.get_logger().warn(
                        "[TUNE-LIFT] Lift cartesian failed — trying joint fallback."
                    )
                    if not self.plan_and_execute_pose(shover, cartesian=False):
                        self.get_logger().error(
                            "[TUNE-LIFT] FAILED to lift back to hover."
                        )
                        return

                self.get_logger().info(
                    f"[TUNE-LIFT] Hover reached at {tune_square}. Tuning complete."
                )
                return

        # ---------------------------------------------------
        # Unknown mode
        # ---------------------------------------------------
        self.get_logger().error(
            f"[TUNE] Unknown tune_mode '{tune_mode}'. "
            f"Use one of: center, square, lift, hold, open, home."
        )
# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)

    # Create executor capable of spinning callbacks from Arduino thread
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    # Spin executor in background thread
    threading.Thread(target=executor.spin, daemon=True).start()

    # Determine task
    task = node.get_parameter("task").get_parameter_value().string_value
    node.get_logger().info(f"Executing task: '{task}'")

    try:
        # ---------------------------------------------------------
        # SELECT TASK
        # ---------------------------------------------------------
        if task == "tune":
            node.task_tune()

        elif task == "home":
            node.move_to_joints(node.j_home)

        else:
            node.get_logger().warn(
                "Unknown task. Valid tasks: tune, home"
            )

    finally:
        # ---------------------------------------------------------
        # CLEAN SHUTDOWN
        # ---------------------------------------------------------
        try:
            if node.arduino:
                node.arduino.stop()
        except Exception:
            pass

        time.sleep(1.0)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
