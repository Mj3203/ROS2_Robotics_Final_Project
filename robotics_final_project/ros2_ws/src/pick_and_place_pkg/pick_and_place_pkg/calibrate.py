#!/usr/bin/env python3
"""
calibrate.py
Interactive per-square calibration. Visits every square in the same order as snake_test.py's
snake pattern; at each reachable square it descends to board level and prompts for a
left/right and up/down correction (in inches) instead of relying on a single global
rotation/origin fix.

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
from pick_and_place_pkg.snake_test import generate_snake_pattern


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
    # An optional left/right + up/down correction (inches) is applied on top of the square's base
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
    # Uses the same confirmed correction as the last descend, so the lift starts from directly above it.
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

    # Prompts for the left/right and up/down corrections at the current square, returning inches.
    def prompt_corrections(self, square: str) -> tuple:
        lr_raw = input(f"Left/Right for {square} (inches, positive = toward h, negative = toward a, 0 if correct): ")
        ud_raw = input(f"Up/Down for {square} (inches, positive = toward black, negative = toward white, 0 if correct): ")

        try:
            lr_inches = float(lr_raw)
        except ValueError:
            print("Not a number — treating as 0.")
            lr_inches = 0.0

        try:
            ud_inches = float(ud_raw)
        except ValueError:
            print("Not a number — treating as 0.")
            ud_inches = 0.0

        return lr_inches, ud_inches

    # Visits every square in snake order, descending and prompting for a correction at each reachable
    # one, then prints a summary of skipped squares and a paste-ready per-square correction dict.
    def run_calibration(self):
        squares = generate_snake_pattern()
        total = len(squares)
        self.get_logger().info(f"Starting per-square calibration across {total} squares.")

        results = {}
        skipped = []

        for i, square in enumerate(squares):
            self.get_logger().info(f"[{i + 1}/{total}] Visiting {square}.")

            if not self.hover_and_descend(square):
                self.get_logger().warn(f"{square} is unreachable — skipping.")
                skipped.append(square)
                continue

            lr_inches, ud_inches = self.prompt_corrections(square)

            # A nonzero correction means the person wants to see it applied before confirming — keep
            # redescending to the accumulated total and asking again until they answer "y".
            if lr_inches != 0.0 or ud_inches != 0.0:
                while True:
                    if not self.hover_and_descend(square, lr_inches, ud_inches):
                        self.get_logger().error(f"Failed to move to corrected position for {square}. Keeping last entered correction.")
                        break

                    confirm = input("Confirm centered now? (y/n): ").strip().lower()
                    if confirm == "y":
                        break

                    extra_lr, extra_ud = self.prompt_corrections(square)
                    lr_inches += extra_lr
                    ud_inches += extra_ud

            results[square] = (lr_inches, ud_inches)

            self.lift_to_hover(square, lr_inches, ud_inches)

        self.print_summary(results, skipped)

    # Prints the skipped-square list and a paste-ready PER_SQUARE_CORRECTIONS_INCHES dict literal.
    def print_summary(self, results: dict, skipped: list):
        self.get_logger().info("=" * 60)
        self.get_logger().info("CALIBRATION SUMMARY")
        self.get_logger().info("=" * 60)

        if skipped:
            self.get_logger().warn(f"Skipped (unreachable) squares: {skipped}")
        else:
            self.get_logger().info("No squares were skipped.")

        print("\nPER_SQUARE_CORRECTIONS_INCHES = {")
        for col in "abcdefgh":
            for row in "12345678":
                square = f"{col}{row}"
                if square in results:
                    lr, ud = results[square]
                    print(f"    '{square}': ({lr}, {ud}),")
                else:
                    print(f"    # '{square}': SKIPPED - unreachable")
        print("}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CalibrateNode()
    except Exception as e:
        print(f"Failed to initialize CalibrateNode: {e}")
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        node.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
