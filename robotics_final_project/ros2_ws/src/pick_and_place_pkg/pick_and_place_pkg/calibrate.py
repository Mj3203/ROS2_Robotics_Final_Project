#!/usr/bin/env python3
"""
calibrate.py
Interactive two-phase calibration, 16 stops total. Phase 1 sweeps row 1 (a1 through h1),
one stop per column, to find a left/right adjustment per column. Phase 2 sweeps column a
(a1 through a8), one stop per row, to find an up/down adjustment per row. At each stop the
arm descends to board level and prompts for a single adjustment instead of relying on a
single global rotation/origin fix.

Run:
    ros2 run pick_and_place_pkg calibrate
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose
from pick_and_place_pkg.constants import (
    BOARD_Z, HOVER_ABOVE_BOARD_M, PLACE_Z_OFFSET, TOP_DOWN, INCH,
    make_pose, square_center_in_world,
)

COLUMNS = "abcdefgh"
ROWS = "12345678"


class CalibrateNode(Node):

    # Initializes the node, sets up MoveIt, and waits for it to become ready.
    def __init__(self):
        super().__init__("calibrate_node")
        self.setup_moveit()
        self.wait_for_moveit()
        self.get_logger().info("CalibrateNode is ready.")

    # Creates the MoveIt2 interface for the arm, matching snake_test.py's setup.
    def setup_moveit(self):
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
            use_move_group_action=True,
        )

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

    # Plans a path to a pose (joint or cartesian) and executes it, returning success.
    def plan_and_execute_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        try:
            traj = self.moveit2.plan(
                pose=pose,
                cartesian=cartesian,
                max_step=0.005,
                cartesian_fraction_threshold=0.85 if cartesian else None,
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

    # Moves to hover above a square then descends to board level, mirroring snake_test.py's
    # visit_square hover/descend steps (joint-then-cartesian for hover, cartesian-only for descend).
    # An optional left/right + up/down adjustment (inches) is applied on top of the square's base
    # world position, so this same method can redescend to a corrected spot during confirmation.
    def hover_and_descend(self, square: str, lr_inches: float = 0.0, ud_inches: float = 0.0) -> bool:
        center = square_center_in_world(square)
        hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
        pick_z = BOARD_Z + PLACE_Z_OFFSET

        x = center.position.x + lr_inches * INCH
        y = center.position.y + ud_inches * INCH

        hover_pose = make_pose(x, y, hover_z, *TOP_DOWN)
        pick_pose = make_pose(x, y, pick_z, *TOP_DOWN)

        self.get_logger().info(f"Moving to hover above {square}.")
        if not self.plan_and_execute_pose(hover_pose, cartesian=False):
            if not self.plan_and_execute_pose(hover_pose, cartesian=True):
                self.get_logger().error(f"Failed to hover above {square}.")
                return False

        self.get_logger().info(f"Descending to {square}.")
        if not self.plan_and_execute_pose(pick_pose, cartesian=True):
            self.get_logger().error(f"Failed to descend to {square}. Lifting back.")
            self.plan_and_execute_pose(hover_pose, cartesian=True)
            return False

        time.sleep(0.1)
        return True

    # Lifts back to hover above a square before moving on, mirroring snake_test.py's lift step.
    # Uses the same confirmed adjustment as the last descend, so the lift starts from directly above it.
    def lift_to_hover(self, square: str, lr_inches: float = 0.0, ud_inches: float = 0.0) -> bool:
        center = square_center_in_world(square)
        hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
        x = center.position.x + lr_inches * INCH
        y = center.position.y + ud_inches * INCH
        hover_pose = make_pose(x, y, hover_z, *TOP_DOWN)

        self.get_logger().info(f"Lifting back from {square}.")
        if self.plan_and_execute_pose(hover_pose, cartesian=True):
            return True
        if self.plan_and_execute_pose(hover_pose, cartesian=False):
            return True
        self.get_logger().error(f"Failed to lift from {square}.")
        return False

    # Prompts for a column's left/right adjustment, returning inches.
    def prompt_column_adjustment(self, col: str) -> float:
        raw = input(f"Left/Right adjustment for column {col} (inches, positive = toward h, negative = toward a, 0 if this column is already fine): ")
        try:
            return float(raw)
        except ValueError:
            print("Not a number — treating as 0.")
            return 0.0

    # Prompts for a row's up/down adjustment, returning inches.
    def prompt_row_adjustment(self, row: str) -> float:
        raw = input(f"Up/Down adjustment for row {row} (inches, positive = toward black, negative = toward white, 0 if this row is already fine): ")
        try:
            return float(raw)
        except ValueError:
            print("Not a number — treating as 0.")
            return 0.0

    # Visits <col>1, prompts for a left/right adjustment, and confirms-and-redescends until accepted.
    def calibrate_column(self, col: str) -> float:
        square = f"{col}1"
        self.get_logger().info(f"Visiting {square} for column calibration.")
        if not self.hover_and_descend(square):
            self.get_logger().warn(f"{square} is unreachable — column '{col}' left at 0.0.")
            return 0.0

        lr_inches = self.prompt_column_adjustment(col)

        if lr_inches != 0.0:
            while True:
                if not self.hover_and_descend(square, lr_inches, 0.0):
                    self.get_logger().error(f"Failed to move to corrected position for {square}. Keeping last entered adjustment.")
                    break

                confirm = input("Confirm centered now? (y/n): ").strip().lower()
                if confirm == "y":
                    break

                lr_inches += self.prompt_column_adjustment(col)

        self.lift_to_hover(square, lr_inches, 0.0)
        return lr_inches

    # Visits a<row>, prompts for an up/down adjustment, and confirms-and-redescends until accepted.
    def calibrate_row(self, row: str) -> float:
        square = f"a{row}"
        self.get_logger().info(f"Visiting {square} for row calibration.")
        if not self.hover_and_descend(square):
            self.get_logger().warn(f"{square} is unreachable — row '{row}' left at 0.0.")
            return 0.0

        ud_inches = self.prompt_row_adjustment(row)

        if ud_inches != 0.0:
            while True:
                if not self.hover_and_descend(square, 0.0, ud_inches):
                    self.get_logger().error(f"Failed to move to corrected position for {square}. Keeping last entered adjustment.")
                    break

                confirm = input("Confirm centered now? (y/n): ").strip().lower()
                if confirm == "y":
                    break

                ud_inches += self.prompt_row_adjustment(row)

        self.lift_to_hover(square, 0.0, ud_inches)
        return ud_inches

    # Runs both calibration phases in sequence, then prints the two adjustment dicts.
    def run_calibration(self):
        self.get_logger().info("=== Phase 1: column calibration (row 1) ===")
        column_adjust = {}
        for col in COLUMNS:
            column_adjust[col] = self.calibrate_column(col)

        self.get_logger().info("=== Phase 2: row calibration (column a) ===")
        row_adjust = {}
        for row in ROWS:
            row_adjust[row] = self.calibrate_row(row)

        self.print_summary(column_adjust, row_adjust)

    # Prints both adjustment dicts, labeled, with 0.0 clearly called out as "no change needed".
    def print_summary(self, column_adjust: dict, row_adjust: dict):
        self.get_logger().info("=" * 60)
        self.get_logger().info("CALIBRATION SUMMARY")
        self.get_logger().info("=" * 60)

        print("\n=== Column adjustments (apply to all squares in that column) ===")
        print("0.0 means no change needed for that column.")
        column_entries = ", ".join(f"'{col}': {column_adjust[col]}" for col in COLUMNS)
        print(f"column_adjust = {{{column_entries}}}")

        print("\n=== Row adjustments (apply to all squares in that row) ===")
        print("0.0 means no change needed for that row.")
        row_entries = ", ".join(f"'{row}': {row_adjust[row]}" for row in ROWS)
        print(f"row_adjust = {{{row_entries}}}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CalibrateNode()
    except Exception as e:
        print(f"Failed to initialize CalibrateNode: {e}")
        rclpy.try_shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        node.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
