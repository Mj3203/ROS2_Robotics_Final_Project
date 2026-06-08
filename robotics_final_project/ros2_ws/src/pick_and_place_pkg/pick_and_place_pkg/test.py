#!/usr/bin/env python3
"""
ChessMoverNode
Robotic chess piece mover controlled via MoveIt2 and optional Arduino solenoid.
This file implements:
    MoveIt2-based pose planning/execution wrappers
    Simple Arduino serial client (non-fatal when pyserial missing)
    High-level tasks: demo move, param-driven move, and tuning helpers
"""

import math
import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface

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
# Tunable defaults and units
# -------------------------
SOLENOID_OFF_DELAY_DEFAULT = 0.2  # seconds (post-OFF wait)

INCH = 0.0254
HOVER_ABOVE_BOARD_M = 2.0 * INCH
CENTER_HOVER_ABOVE_BOARD_M = 8.0 * INCH
PLACE_Z_OFFSET = 0.0
BOARD_Z = 8.4 * INCH
BOARD_ORIGIN = (-6.55 * INCH, 7.4 * INCH, BOARD_Z)
TOP_DOWN = (-0.7071, 0.7071, 0.0, 0.0)

# -------------------------
# Square coordinates (meters, relative to board origin)
# -------------------------
SQUARE_COORDS_M = {
    'A1': (0.020637, 0.020637), 'A2': (0.020637, 0.061912), 'A3': (0.020637, 0.103188),
    'A4': (0.020637, 0.144462), 'A5': (0.020637, 0.185737), 'A6': (0.020637, 0.227013),
    'A7': (0.020637, 0.268287), 'A8': (0.020637, 0.309563),
    'B1': (0.061912, 0.020637), 'B2': (0.061912, 0.061912), 'B3': (0.061912, 0.103188),
    'B4': (0.061912, 0.144462), 'B5': (0.061912, 0.185737), 'B6': (0.061912, 0.227013),
    'B7': (0.061912, 0.268287), 'B8': (0.061912, 0.309563),
    'C1': (0.103188, 0.020637), 'C2': (0.103188, 0.061912), 'C3': (0.103188, 0.103188),
    'C4': (0.103188, 0.144462), 'C5': (0.103188, 0.185737), 'C6': (0.103188, 0.227013),
    'C7': (0.103188, 0.268287), 'C8': (0.103188, 0.309563),
    'D1': (0.144462, 0.020637), 'D2': (0.144462, 0.061912), 'D3': (0.144462, 0.103188),
    'D4': (0.144462, 0.144462), 'D5': (0.144462, 0.185737), 'D6': (0.144462, 0.227013),
    'D7': (0.144462, 0.268287), 'D8': (0.144462, 0.309563),
    'E1': (0.185737, 0.020637), 'E2': (0.185737, 0.061912), 'E3': (0.185737, 0.103188),
    'E4': (0.185737, 0.144462), 'E5': (0.185737, 0.185737), 'E6': (0.185737, 0.227013),
    'E7': (0.185737, 0.268287), 'E8': (0.185737, 0.309563),
    'F1': (0.227013, 0.020637), 'F2': (0.227013, 0.061912), 'F3': (0.227013, 0.103188),
    'F4': (0.227013, 0.144462), 'F5': (0.227013, 0.185737), 'F6': (0.227013, 0.227013),
    'F7': (0.227013, 0.268287), 'F8': (0.227013, 0.309563),
    'G1': (0.268287, 0.020637), 'G2': (0.268287, 0.061912), 'G3': (0.268287, 0.103188),
    'G4': (0.268287, 0.144462), 'G5': (0.268287, 0.185737), 'G6': (0.268287, 0.227013),
    'G7': (0.268287, 0.268287), 'G8': (0.268287, 0.309563),
    'H1': (0.309563, 0.020637), 'H2': (0.309563, 0.061912), 'H3': (0.309563, 0.103188),
    'H4': (0.309563, 0.144462), 'H5': (0.309563, 0.185737), 'H6': (0.309563, 0.227013),
    'H7': (0.309563, 0.268287), 'H8': (0.309563, 0.309563),
}

# -------------------------
# Helpers
# -------------------------

def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    """Create and return a geometry_msgs/Pose from numeric components."""
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p


def square_center_in_world(square: str) -> Pose:
    """Return a Pose at the center of the given board square in world coordinates."""
    sq = square.upper()
    if sq not in SQUARE_COORDS_M:
        raise ValueError(f"Unknown square {sq}")
    x_rel, y_rel = SQUARE_COORDS_M[sq]
    bx, by, bz = BOARD_ORIGIN
    return make_pose(bx + x_rel, by + y_rel, bz, *TOP_DOWN)


def compute_board_center():
    """Compute and return the board center (x, y, z) in world coordinates."""
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
    Lightweight Arduino client:
    - Background reader thread for Arduino messages (non-fatal if pyserial missing)
    - Thread-safe send(line) method that appends newline
    """

    def __init__(self, node: Node, port: str = "/dev/ttyUSB0", baud: int = 115200, timeout: float = 1.0):
        self.node = node
        self.port = port
        self.baud = int(baud)
        self.timeout = float(timeout)
        self._ser = None
        self._reader_thread = None
        self._running = False
        self._lock = threading.Lock()

        if not SERIAL_AVAILABLE:
            self.node.get_logger().warn("pyserial not available. Arduino integration disabled.")
            return
        self._start()

    def _start(self):
        """Start the background reader thread."""
        self._running = True
        self._reader_thread = threading.Thread(target=self._run_reader, daemon=True)
        self._reader_thread.start()

    def _open_serial(self):
        """Attempt to open the configured serial port; return serial.Serial or None on failure."""
        try:
            s = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.2)
            self.node.get_logger().info(f"ArduinoSerialClient: opened {self.port} @ {self.baud}")
            return s
        except Exception as e:
            self.node.get_logger().warn(f"ArduinoSerialClient: cannot open {self.port}: {e}")
            return None

    def _run_reader(self):
        """Background loop that reads lines from Arduino and logs them at DEBUG level."""
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
                else:
                    time.sleep(0.05)
            except Exception as e:
                self.node.get_logger().warn(f"ArduinoSerialClient read error: {e} -- closing and will retry")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(reconnect_delay)

    def send(self, line: str) -> bool:
        """
        Send a single line to Arduino (appends newline). Thread-safe.
        Returns True when the write was attempted successfully, False otherwise.
        """
        if not SERIAL_AVAILABLE:
            self.node.get_logger().debug("ArduinoSerialClient.send() called but pyserial missing.")
            return False
        with self._lock:
            if self._ser is None:
                self._ser = self._open_serial()
            if self._ser is None:
                self.node.get_logger().warn("ArduinoSerialClient: not connected; cannot send command.")
                return False
            try:
                msg = (line.strip() + '\n').encode()
                self._ser.write(msg)
                self._ser.flush()
                self.node.get_logger().debug(f"ArduinoSerialClient: sent -> {line}")
                return True
            except Exception as e:
                self.node.get_logger().warn(f"ArduinoSerialClient send failed: {e}")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                return False

    def stop(self):
        """Stop reader thread and close serial port if open."""
        self._running = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


# -------------------------
# Main ROS node
# -------------------------

class ChessMoverNode(Node):
    """Main node that wraps MoveIt2, optional gripper, Arduino client, and high-level tasks."""

    def __init__(self):
        super().__init__("chess_mover_node")

        # Basic parameters (defaults)
        self.declare_parameter("task", "move_B3_B5")
        self.declare_parameter("cartesian_fraction_threshold", 0.90)
        self.declare_parameter("tune_mode", "center")
        self.declare_parameter("tune_square", "B3")
        self.declare_parameter("src", "B3")
        self.declare_parameter("dst", "B5")

        # Arduino-related parameters
        self.declare_parameter("arduino_enabled", True)
        self.declare_parameter("arduino_port", "/dev/ttyUSB0")
        self.declare_parameter("arduino_baud", 115200)
        self.declare_parameter("solenoid_off_delay", float(SOLENOID_OFF_DELAY_DEFAULT))
        self.declare_parameter("auto_detect_arduino", True)

        # MoveIt2 wrapper initialization
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        # Gripper interface (optional)
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
            self.get_logger().warn(f"GripperInterface init failed (gripper functions will be skipped): {e}")

        # Gripper state and home joints
        self.gripper_closed = False
        self.gripper_locked = False
        self.j_home = [0.0] * 6

        # Arduino client initialization with auto-detect logic
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
                                self.get_logger().info(f"Auto-detected Arduino port: {chosen_port} ({p.description})")
                                break
                except Exception as e:
                    self.get_logger().debug(f"Auto-detect attempt failed: {e}")

            if enabled:
                self.arduino = ArduinoSerialClient(self, port=chosen_port, baud=baud)
                self.get_logger().info(f"Arduino client enabled on {chosen_port}@{baud}")
            else:
                self.get_logger().info("Arduino integration disabled via parameter 'arduino_enabled'.")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize Arduino client: {e}")
            self.arduino = None

        time.sleep(0.1)

    # -------------------------
    # Utility: Ensure planner/joint state readiness
    # -------------------------

    def wait_for_moveit_and_state(self, timeout_s: int = 60) -> bool:
        """Wait for MoveIt planner service and /joint_states to be available."""
        import subprocess
        self.get_logger().info(f"Waiting up to {timeout_s}s for planner and /joint_states...")
        start = time.time()

        planner_ready = False
        while time.time() - start < timeout_s:
            try:
                out = subprocess.run(["ros2", "service", "list"], capture_output=True, text=True, timeout=3).stdout
            except subprocess.TimeoutExpired:
                out = ""
            if "/plan_kinematic_path" in out:
                planner_ready = True
                break
            time.sleep(0.5)

        if not planner_ready:
            self.get_logger().warn("Timed out waiting for planner service.")

        expected = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
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
                all_present = all(j in joint_out for j in expected)
                if all_present:
                    joint_ok = True
                    break
            time.sleep(0.3)

        if not joint_ok:
            self.get_logger().warn("Timed out waiting for arm joint_states (robot state may be stale).")

        ok = planner_ready and joint_ok
        if ok:
            self.get_logger().info("Planner and joint_states detected.")
        return ok

    # -------------------------
    # Move wrappers
    # -------------------------

    def move_to_joints(self, joints) -> bool:
        """Move the arm to a specific joint configuration (blocking)."""
        try:
            self.get_logger().info(f"Moving to joints: {['{:.3f}'.format(p) for p in joints]}")
            self.moveit2.move_to_configuration(joint_positions=joints)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"move_to_configuration failed: {e}")
            return False

    def plan_and_execute_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        """
        Plan (and execute) a pose with MoveIt2.
        If cartesian=True, request Cartesian planning; use configured cartesian_fraction_threshold.
        """
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
                cartesian_fraction_threshold=threshold if cartesian else None,
            )
        except Exception as e:
            self.get_logger().warn(f"planning exception: {e}")
            return False

        if traj is None:
            self.get_logger().warn(
                f"Planning to pose failed (cartesian={cartesian}). "
                f"Target: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}"
            )
            return False

        try:
            self.moveit2.execute(traj)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().warn(f"Execution failed after planning: {e}")
            return False

    # -------------------------
    # Gripper helpers
    # -------------------------

    def lock_gripper(self):
        """Prevent future open() calls on the gripper by replacing the method with a blocker."""
        if self.gripper is None:
            self.get_logger().warn("lock_gripper() requested but no gripper interface present.")
            self.gripper_locked = True
            return
        if self.gripper_locked:
            self.get_logger().info("Gripper already locked.")
            return
        try:
            if not hasattr(self.gripper, "_original_open"):
                self.gripper._original_open = getattr(self.gripper, "open", None)
        except Exception:
            self.get_logger().debug("Could not store original gripper.open (non-fatal).")

        def _blocked_open(*args, **kwargs):
            self.get_logger().warn("Gripper open request blocked because gripper is locked by 'hold' tuning mode.")
            return False

        try:
            setattr(self.gripper, "open", _blocked_open)
            self.gripper_locked = True
            self.get_logger().info("Gripper locked: future open() calls will be blocked.")
        except Exception as e:
            self.get_logger().warn(f"Failed to lock gripper (non-fatal): {e}")
            self.gripper_locked = True

    def open_gripper_now(self) -> bool:
        """Open the gripper immediately (if present and not locked)."""
        if self.gripper_locked:
            self.get_logger().warn("open_gripper_now() refused: gripper is locked.")
            return False
        if self.gripper is None:
            self.get_logger().warn("No gripper interface available; skipping open_gripper.")
            return False
        try:
            self.get_logger().info("Opening gripper now.")
            self.gripper.open()
            time.sleep(0.15)
            self.gripper_closed = False
            return True
        except Exception as e:
            self.get_logger().warn(f"Gripper open failed: {e}")
            return False

    def close_gripper_now(self) -> bool:
        """Close the gripper immediately (if present)."""
        if self.gripper is None:
            self.get_logger().warn("No gripper interface available; skipping close_gripper.")
            self.gripper_closed = True
            return False
        try:
            self.get_logger().info("Closing gripper now.")
            self.gripper.close()
            time.sleep(0.15)
            self.gripper_closed = True
            return True
        except Exception as e:
            self.get_logger().warn(f"Gripper close failed: {e}")
            return False

    # -------------------------
    # High-level move: move a piece from src -> dst
    # -------------------------

    def move_piece(self, src: str, dst: str, piece_object_id: str = None) -> bool:
        """
        Move a piece from src to dst using hover, pick, travel, place sequence.
        Uses the Arduino client to toggle solenoid at appropriate moments (if present).
        """
        src = src.upper()
        dst = dst.upper()

        if src not in SQUARE_COORDS_M or dst not in SQUARE_COORDS_M:
            self.get_logger().error("Invalid squares passed to move_piece.")
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

        # Hover to source
        if not self.plan_and_execute_pose(shover, cartesian=False):
            self.get_logger().warn("Hover to source (joint-space) failed — trying small offsets...")
            success = False
            for dx, dy in [(0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)]:
                trial = make_pose(shover.position.x + dx, shover.position.y + dy, shover.position.z, *TOP_DOWN)
                if self.plan_and_execute_pose(trial, cartesian=False):
                    success = True
                    break
            if not success:
                self.get_logger().error("Failed to reach source hover.")
                return False
        time.sleep(0.08)

        # Solenoid ON
        try:
            if self.arduino:
                ok = self.arduino.send("sol on")
                if ok:
                    self.get_logger().info("Sent solenoid ON command to Arduino (at source hover).")
                else:
                    self.get_logger().warn("Attempted to send solenoid ON but Arduino client not connected.")
            else:
                self.get_logger().debug("Arduino client not initialized; skipping solenoid ON.")
        except Exception as e:
            self.get_logger().warn(f"Error while sending solenoid ON: {e}")

        # Descend to pick
        if not self.plan_and_execute_pose(spick, cartesian=True):
            self.get_logger().warn("Direct Cartesian descend failed; attempting staged descent.")
            ok_descend = True
            for i in range(1, 4):
                z_step = shover.position.z + (spick.position.z - shover.position.z) * (i / 3)
                step_pose = make_pose(spick.position.x, spick.position.y, z_step, *TOP_DOWN)
                if not self.plan_and_execute_pose(step_pose, cartesian=True):
                    ok_descend = False
                    break
                time.sleep(0.05)
            if not ok_descend:
                self.get_logger().error("Failed staged descent to pick.")
                return False

        # Attach object in planning scene
        if piece_object_id:
            if self.gripper_closed:
                try:
                    self.moveit2.attach_collision_object(
                        piece_object_id, "end_effector_link",
                        touch_links=["right_finger_bottom_joint"]
                    )
                    time.sleep(0.02)
                except Exception as e:
                    self.get_logger().warn(f"Attach object failed (non-fatal): {e}")
            else:
                self.get_logger().warn("Skipping attach of object in planning scene because gripper is not closed.")

        # Lift back to hover
        if not self.plan_and_execute_pose(shover, cartesian=True):
            self.get_logger().warn("Lift after pick failed (trying joint-space).")
            if not self.plan_and_execute_pose(shover, cartesian=False):
                self.get_logger().error("Failed to lift after pick.")
                return False
        time.sleep(0.08)

        # Travel to target hover
        dist_xy = math.hypot(shover.position.x - thover.position.x, shover.position.y - thover.position.y)
        prefer_cart = dist_xy < 0.08
        if prefer_cart:
            if not self.plan_and_execute_pose(thover, cartesian=True):
                self.get_logger().warn("Cartesian travel to target hover failed; trying joint-space fallback.")
                if not self.plan_and_execute_pose(thover, cartesian=False):
                    self.get_logger().error("Failed to travel to target hover.")
                    return False
        else:
            if not self.plan_and_execute_pose(thover, cartesian=False):
                self.get_logger().warn("Joint-space travel to target hover failed; trying Cartesian fallback.")
                if not self.plan_and_execute_pose(thover, cartesian=True):
                    self.get_logger().error("Failed to travel to target hover.")
                    return False
        time.sleep(0.08)

        # Descend to place
        if not self.plan_and_execute_pose(tplace, cartesian=True):
            self.get_logger().warn("Direct Cartesian descend to place failed; trying staged.")
            ok_place = True
            for i in range(1, 4):
                z_step = thover.position.z + (tplace.position.z - thover.position.z) * (i / 3)
                step_pose = make_pose(tplace.position.x, tplace.position.y, z_step, *TOP_DOWN)
                if not self.plan_and_execute_pose(step_pose, cartesian=True):
                    ok_place = False
                    break
                time.sleep(0.05)
            if not ok_place:
                self.get_logger().error("Failed staged descend to place.")
                return False

        # Detach object
        if piece_object_id:
            try:
                self.moveit2.detach_collision_object(piece_object_id)
                time.sleep(0.02)
            except Exception as e:
                self.get_logger().warn(f"Detach object failed (non-fatal): {e}")

        # Solenoid OFF
        try:
            if self.arduino:
                ok = self.arduino.send("sol off")
                if ok:
                    self.get_logger().info("Sent solenoid OFF command to Arduino (after place).")
                else:
                    self.get_logger().warn("Attempted to send solenoid OFF but Arduino client not connected.")
            else:
                self.get_logger().debug("Arduino client not initialized; skipping solenoid OFF.")
        except Exception as e:
            self.get_logger().warn(f"Error while sending solenoid OFF: {e}")

        try:
            sol_off_delay = float(self.get_parameter("solenoid_off_delay").get_parameter_value().double_value)
        except Exception:
            sol_off_delay = float(SOLENOID_OFF_DELAY_DEFAULT)
        if sol_off_delay > 0.0:
            self.get_logger().info(f"Waiting {sol_off_delay:.3f}s AFTER solenoid OFF to ensure piece settled.")
            time.sleep(sol_off_delay)

        # Retract to hover
        if not self.plan_and_execute_pose(thover, cartesian=True):
            self.get_logger().warn("Retract to hover after place failed.")
            self.plan_and_execute_pose(thover, cartesian=False)

        return True

    # -------------------------
    # Tasks exposed via parameters
    # -------------------------

    def task_move_B3_B5(self):
        """Convenience demo task: set src/dst and run task_move_from_params."""
        self.get_logger().info("Running demo move_B3_B5 (B3 -> B5).")
        self.set_parameters([
            rclpy.parameter.Parameter('src', rclpy.Parameter.Type.STRING, 'B3'),
            rclpy.parameter.Parameter('dst', rclpy.Parameter.Type.STRING, 'B5'),
            rclpy.parameter.Parameter('task', rclpy.Parameter.Type.STRING, 'move_B3_B5'),
        ])
        return self.task_move_from_params()

    def task_move_from_params(self):
        """
        Move according to node parameters:
        - Ensures MoveIt/joint_states (best-effort)
        - Moves REST -> ARC -> CENTER for smooth approach
        - Calls move_piece(src, dst)
        - Returns via CENTER -> ARC -> REST and ends at REST
        """
        ok = self.wait_for_moveit_and_state(timeout_s=60)
        if not ok:
            self.get_logger().warn("Planner/joint_states not fully ready; attempting moves anyway.")

        src = self.get_parameter("src").get_parameter_value().string_value
        dst = self.get_parameter("dst").get_parameter_value().string_value

        if not src or not dst:
            self.get_logger().error("Both 'src' and 'dst' parameters are required (e.g. src:=B3 dst:=B5).")
            return

        src = src.upper()
        dst = dst.upper()

        if src not in SQUARE_COORDS_M:
            self.get_logger().error(f"Invalid src square '{src}'.")
            return
        if dst not in SQUARE_COORDS_M:
            self.get_logger().error(f"Invalid dst square '{dst}'.")
            return

        self.get_logger().info("NOTE: Not moving to 'home' first; operating from current robot pose.")

        cx, cy, cz = compute_board_center()
        center_hover_z = cz + CENTER_HOVER_ABOVE_BOARD_M
        center_hover_pose = make_pose(cx, cy, center_hover_z, *TOP_DOWN)

        rest_x = cy
        rest_y = cx
        rest_pose = make_pose(rest_x, rest_y, center_hover_z, *TOP_DOWN)

        arc_mid_x = (rest_x + cx) / 2.0
        arc_mid_y = (rest_y + cy) / 2.0
        arc_z = center_hover_z + 0.05
        arc_pose = make_pose(arc_mid_x, arc_mid_y, arc_z, *TOP_DOWN)

        # Move to REST
        self.get_logger().info(f"Moving to REST POSE at x={rest_x:.3f}, y={rest_y:.3f}, z={center_hover_z:.3f}")
        if not self.plan_and_execute_pose(rest_pose, cartesian=False):
            self.get_logger().warn("Rest pose joint-space failed; trying cartesian fallback.")
            if not self.plan_and_execute_pose(rest_pose, cartesian=True):
                self.get_logger().error("Failed to reach rest pose. Aborting.")
                return
        time.sleep(0.2)

        # REST -> ARC
        self.get_logger().info(f"Moving from REST to ARC at x={arc_mid_x:.3f}, y={arc_mid_y:.3f}, z={arc_z:.3f}")
        if not self.plan_and_execute_pose(arc_pose, cartesian=True):
            self.get_logger().warn("Arc pose cartesian failed; trying joint-space fallback.")
            if not self.plan_and_execute_pose(arc_pose, cartesian=False):
                self.get_logger().error("Failed to reach arc pose. Aborting.")
                self.plan_and_execute_pose(rest_pose, cartesian=False)
                return
        time.sleep(0.12)

        # ARC -> CENTER
        self.get_logger().info(f"Moving from ARC to CENTER HOVER at x={cx:.3f}, y={cy:.3f}, z={center_hover_z:.3f}")
        if not self.plan_and_execute_pose(center_hover_pose, cartesian=True):
            self.get_logger().warn("Center hover cartesian failed; trying joint-space fallback.")
            if not self.plan_and_execute_pose(center_hover_pose, cartesian=False):
                self.get_logger().error("Failed to reach center hover from arc pose. Aborting.")
                self.plan_and_execute_pose(rest_pose, cartesian=False)
                return
        time.sleep(0.2)

        # Perform the piece move
        try:
            if not self.move_piece(src, dst, piece_object_id=None):
                self.get_logger().error(f"Move {src}->{dst} failed.")
                self.plan_and_execute_pose(center_hover_pose, cartesian=False)
                self.plan_and_execute_pose(rest_pose, cartesian=False)
                return
        except Exception as e:
            self.get_logger().error(f"Exception during move_piece: {e}")
            self.plan_and_execute_pose(center_hover_pose, cartesian=False)
            self.plan_and_execute_pose(rest_pose, cartesian=False)
            return

        # Return to CENTER
        self.get_logger().info("Returning to CENTER HOVER to finish.")
        if not self.plan_and_execute_pose(center_hover_pose, cartesian=False):
            self.get_logger().warn("Failed to return to center hover via joint-space; trying cartesian fallback.")
            self.plan_and_execute_pose(center_hover_pose, cartesian=True)

        # CENTER -> ARC -> REST
        self.get_logger().info(f"Moving back to ARC at x={arc_mid_x:.3f}, y={arc_mid_y:.3f}, z={arc_z:.3f}")
        if not self.plan_and_execute_pose(arc_pose, cartesian=True):
            self.get_logger().warn("Arc pose move (return leg) failed; trying joint-space fallback.")
            self.plan_and_execute_pose(arc_pose, cartesian=False)
        time.sleep(0.12)

        self.get_logger().info(f"Moving back to REST at x={rest_x:.3f}, y={rest_y:.3f}, z={center_hover_z:.3f}")
        if not self.plan_and_execute_pose(rest_pose, cartesian=False):
            self.get_logger().warn("Return to rest pose joint-space failed; trying cartesian fallback.")
            self.plan_and_execute_pose(rest_pose, cartesian=True)

        self.get_logger().info("Completed task_move_from_params (ended at rest pose).")

    def task_tune(self):
        """
        Tuning task with several modes:
        - 'center' : move to board center hover
        - 'square' : move to a square then descend and stop
        - 'lift'   : descend then lift back to hover
        - 'hold'   : close and lock gripper
        - 'open'   : open gripper now
        - 'home'   : move to joint home
        """
        ok = self.wait_for_moveit_and_state(timeout_s=60)
        if not ok:
            self.get_logger().warn("Planner/joint_states not fully ready; attempting moves anyway.")

        tune_mode = self.get_parameter("tune_mode").get_parameter_value().string_value
        tune_mode = tune_mode.lower() if isinstance(tune_mode, str) else "center"

        if tune_mode == "home":
            self.get_logger().info("[TUNE] Moving to home (straight-up) as requested.")
            if not self.move_to_joints(self.j_home):
                self.get_logger().error("[TUNE] Failed to move to home.")
            return

        if tune_mode == "hold":
            self.get_logger().info("[TUNE-HOLD] Closing and locking the gripper.")
            if not self.close_gripper_now():
                self.get_logger().warn("[TUNE-HOLD] close_gripper_now() returned False (non-fatal). Locking anyway.")
            self.lock_gripper()
            self.get_logger().info("[TUNE-HOLD] Gripper closed and locked.")
            return

        if tune_mode == "open":
            self.get_logger().info("[TUNE-OPEN] Requested gripper OPEN now.")
            ok_open = self.open_gripper_now()
            if ok_open:
                self.get_logger().info("[TUNE-OPEN] Gripper opened successfully.")
            else:
                self.get_logger().warn("[TUNE-OPEN] Gripper open failed or was refused.")
            return

        cx, cy, cz = compute_board_center()
        center_hover_z = cz + CENTER_HOVER_ABOVE_BOARD_M
        center_hover_pose = make_pose(cx, cy, center_hover_z, *TOP_DOWN)

        if tune_mode == "center":
            self.get_logger().info(f"[TUNE] Moving to CENTER HOVER at x={cx:.3f}, y={cy:.3f}, z={center_hover_z:.3f}")
            if not self.plan_and_execute_pose(center_hover_pose, cartesian=False):
                self.get_logger().warn("[TUNE] Center hover joint-space failed; trying cartesian fallback.")
                if not self.plan_and_execute_pose(center_hover_pose, cartesian=True):
                    self.get_logger().error("[TUNE] Failed to reach center hover.")
                else:
                    self.get_logger().info("[TUNE] Reached center hover. Tuning complete.")
            return

        if tune_mode in ("square", "lift"):
            tune_square = self.get_parameter("tune_square").get_parameter_value().string_value
            if not tune_square:
                self.get_logger().error("[TUNE] tune_mode requires 'tune_square' parameter.")
                return
            tune_square = tune_square.upper()
            if tune_square not in SQUARE_COORDS_M:
                self.get_logger().error(f"[TUNE] Unknown tune_square '{tune_square}'.")
                return

            s_center = square_center_in_world(tune_square)
            hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
            shover = make_pose(s_center.position.x, s_center.position.y, hover_z, *TOP_DOWN)
            spick = make_pose(s_center.position.x, s_center.position.y, BOARD_Z + PLACE_Z_OFFSET, *TOP_DOWN)

            self.get_logger().info(
                f"[TUNE] Moving to hover over {tune_square} at x={shover.position.x:.3f}, "
                f"y={shover.position.y:.3f}, z={shover.position.z:.3f}"
            )
            if not self.plan_and_execute_pose(shover, cartesian=False):
                self.get_logger().warn("[TUNE] Square hover joint-space failed; trying small offsets then cartesian.")
                success = False
                for dx, dy in [(0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)]:
                    trial = make_pose(shover.position.x + dx, shover.position.y + dy, shover.position.z, *TOP_DOWN)
                    if self.plan_and_execute_pose(trial, cartesian=False):
                        success = True
                        break
                if not success:
                    if not self.plan_and_execute_pose(shover, cartesian=True):
                        self.get_logger().error("[TUNE] Failed to reach square hover.")
                        return
            time.sleep(0.08)

            if tune_mode == "square":
                self.get_logger().info(f"[TUNE] Descending to place at {tune_square} (will NOT lift back up).")
                if not self.plan_and_execute_pose(spick, cartesian=True):
                    self.get_logger().warn("[TUNE] Direct Cartesian descend failed; attempting staged descent.")
                    ok_descend = True
                    for i in range(1, 5):
                        z_step = shover.position.z + (spick.position.z - shover.position.z) * (i / 4)
                        step_pose = make_pose(spick.position.x, spick.position.y, z_step, *TOP_DOWN)
                        if not self.plan_and_execute_pose(step_pose, cartesian=True):
                            ok_descend = False
                            break
                        time.sleep(0.05)
                    if not ok_descend:
                        self.get_logger().error("[TUNE] Failed staged descent to place during tuning.")
                        return
                self.get_logger().info(f"[TUNE] Reached place at {tune_square}. Stopping (no lift).")
                return

            if tune_mode == "lift":
                self.get_logger().info(f"[TUNE-LIFT] Descending to pick/place z at {tune_square}.")
                if not self.plan_and_execute_pose(spick, cartesian=True):
                    self.get_logger().warn("[TUNE-LIFT] Direct Cartesian descend failed; attempting staged descent.")
                    ok_descend = True
                    for i in range(1, 4):
                        z_step = shover.position.z + (spick.position.z - shover.position.z) * (i / 3)
                        step_pose = make_pose(spick.position.x, spick.position.y, z_step, *TOP_DOWN)
                        if not self.plan_and_execute_pose(step_pose, cartesian=True):
                            ok_descend = False
                            break
                        time.sleep(0.05)
                    if not ok_descend:
                        self.get_logger().error("[TUNE-LIFT] Failed staged descent to pick/place.")
                        return
                time.sleep(0.08)

                self.get_logger().info(f"[TUNE-LIFT] Lifting back to hover (z={hover_z:.3f}).")
                if not self.plan_and_execute_pose(shover, cartesian=True):
                    self.get_logger().warn("[TUNE-LIFT] Lift to hover failed (cartesian) - trying joint-space fallback.")
                    if not self.plan_and_execute_pose(shover, cartesian=False):
                        self.get_logger().error("[TUNE-LIFT] Failed to lift back to hover.")
                        return
                self.get_logger().info(f"[TUNE-LIFT] Reached hover over {tune_square}. Tuning lift complete.")
                return

        self.get_logger().error(
            f"[TUNE] Unknown tune_mode '{tune_mode}'. Use 'center', 'square', 'lift', 'hold', 'open', 'home'."
        )


# -------------------------
# main
# -------------------------

def main(args=None):
    """Node bootstrap: initialize rclpy, start executor thread, run requested task, and clean up."""
    rclpy.init(args=args)
    node = ChessMoverNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    task = node.get_parameter("task").get_parameter_value().string_value
    node.get_logger().info(f"Executing task: '{task}'")

    try:
        if task == "move_B3_B5":
            node.task_move_B3_B5()
        elif task == "move_from_params":
            node.task_move_from_params()
        elif task == "tune":
            node.task_tune()
        elif task == "home":
            node.move_to_joints(node.j_home)
        else:
            node.get_logger().warn("Unknown task. Valid tasks: move_B3_B5, move_from_params, tune, home")
    finally:
        try:
            if node.arduino:
                node.arduino.stop()
        except Exception:
            pass
        time.sleep(1)
        rclpy.shutdown()


if __name__ == "__main__":
    main()