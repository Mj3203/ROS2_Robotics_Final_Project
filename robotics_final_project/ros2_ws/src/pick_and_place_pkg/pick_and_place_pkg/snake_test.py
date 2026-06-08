#!/usr/bin/env python3
"""
snake_test.py
Moves the arm in a snake pattern across all 64 squares.
At each square: hover -> descend -> lift back to hover, then move to next square.

Run:
    ros2 run pick_and_place_pkg snake_test
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose
from pick_and_place_pkg.constants import (
    SQUARE_COORDS_M, BOARD_Z, HOVER_ABOVE_BOARD_M,
    PLACE_Z_OFFSET, TOP_DOWN,
    make_pose, square_center_in_world,
)

COLUMNS = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
ROWS = ['1', '2', '3', '4', '5', '6', '7', '8']


def generate_snake_pattern():
    squares = []
    for col_index, col in enumerate(COLUMNS):
        if col_index % 2 == 0:
            rows = ROWS
        else:
            rows = reversed(ROWS)
        for row in rows:
            squares.append(col + row)
    return squares


class SnakeTestNode(Node):

    def __init__(self):
        super().__init__("snake_test_node")
        self.setup_moveit()
        self.wait_for_moveit()
        self.get_logger().info("SnakeTestNode is ready.")

    def setup_moveit(self):
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
            use_move_group_action=True,
        )

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

    def plan_and_execute_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        try:
            traj = self.moveit2.plan(
                pose=pose,
                cartesian=cartesian,
                max_step=0.005,
                cartesian_fraction_threshold=0.90 if cartesian else None,
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

    def visit_square(self, square: str) -> dict:
        center = square_center_in_world(square)
        hover_z = BOARD_Z + HOVER_ABOVE_BOARD_M
        pick_z = BOARD_Z + PLACE_Z_OFFSET

        hover_pose = make_pose(center.position.x, center.position.y, hover_z, *TOP_DOWN)
        pick_pose = make_pose(center.position.x, center.position.y, pick_z, *TOP_DOWN)

        result = {
            "square": square,
            "hover": None,
            "descend": None,
            "lift": None,
            "failed_at": None,
        }

        self.get_logger().info(f"Moving to hover above {square}.")
        if self.plan_and_execute_pose(hover_pose, cartesian=False):
            result["hover"] = "joint"
        elif self.plan_and_execute_pose(hover_pose, cartesian=True):
            result["hover"] = "cartesian"
        else:
            self.get_logger().error(f"Failed to hover above {square}. Skipping.")
            result["failed_at"] = "hover"
            return result

        self.get_logger().info(f"Descending to {square}.")
        if self.plan_and_execute_pose(pick_pose, cartesian=True):
            result["descend"] = "cartesian"
        else:
            self.get_logger().error(f"Failed to descend to {square}. Lifting back.")
            self.plan_and_execute_pose(hover_pose, cartesian=True)
            result["failed_at"] = "descend"
            return result

        time.sleep(0.1)

        self.get_logger().info(f"Lifting back from {square}.")
        if self.plan_and_execute_pose(hover_pose, cartesian=True):
            result["lift"] = "cartesian"
        elif self.plan_and_execute_pose(hover_pose, cartesian=False):
            result["lift"] = "joint"
        else:
            self.get_logger().error(f"Failed to lift from {square}.")
            result["failed_at"] = "lift"
            return result

        return result

    def run_snake(self):
        squares = generate_snake_pattern()
        total = len(squares)
        self.get_logger().info(f"Starting snake pattern across {total} squares.")

        results = []

        for i, square in enumerate(squares):
            self.get_logger().info(f"[{i + 1}/{total}] Visiting {square}.")
            result = self.visit_square(square)
            results.append(result)

        self.get_logger().info("=" * 60)
        self.get_logger().info("SNAKE TEST RESULTS")
        self.get_logger().info("=" * 60)

        failed = []
        for r in results:
            if r["failed_at"] is not None:
                self.get_logger().info(f"{r['square']}: FAILED at {r['failed_at']}")
                failed.append(r["square"])
            else:
                self.get_logger().info(
                    f"{r['square']}: hover={r['hover']}, descend={r['descend']}, lift={r['lift']}"
                )

        self.get_logger().info("=" * 60)
        passed = total - len(failed)
        self.get_logger().info(f"Result: {passed}/{total} squares visited successfully.")
        if failed:
            self.get_logger().warn(f"Failed squares: {failed}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SnakeTestNode()
    except Exception as e:
        print(f"Failed to initialize SnakeTestNode: {e}")
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        node.run_snake()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()