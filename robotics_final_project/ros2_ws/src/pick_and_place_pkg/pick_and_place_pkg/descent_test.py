import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2
from pick_and_place_pkg.constants import (
    TOP_DOWN, make_pose, compute_board_center, CENTER_HOVER_Z
)

# How far to descend in one shot
DESCENT_M = 0.10  # 10cm in one shot


class DescentTestNode(Node):

    def __init__(self):
        super().__init__("descent_test_node")
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
            use_move_group_action=True,
        )
        self.moveit2.allowed_planning_time = 10.0
        self.moveit2.num_planning_attempts = 30

        self.get_logger().info("DescentTestNode ready.")

    def log_joint_states(self, label: str):
        js = self.moveit2.joint_state
        if js is None:
            self.get_logger().warn(f"[{label}] Joint state not available.")
            return
        pairs = list(zip(js.name, js.position))
        pairs_str = ", ".join([f"{n}={v:.3f}rad/{v*57.296:.1f}deg" for n, v in pairs if "joint_" in n])
        self.get_logger().info(f"[{label}] Joints: {pairs_str}")

    def move_to_pose(self, pose, cartesian=False) -> bool:
        self.moveit2.move_to_pose(
            pose=pose,
            cartesian=cartesian,
            cartesian_max_step=0.005,
            cartesian_fraction_threshold=0.90 if cartesian else 0.0,
        )
        self.moveit2.wait_until_executed()
        return self.moveit2.motion_suceeded

    def run_test(self):
        center_x, center_y, _ = compute_board_center()
        center_pose = make_pose(center_x, center_y, CENTER_HOVER_Z, *TOP_DOWN)

        self.get_logger().info(f"Moving to center at z={CENTER_HOVER_Z:.4f} (joint space)...")
        if not self.move_to_pose(center_pose, cartesian=False):
            self.get_logger().error("Failed to reach center. Aborting.")
            return

        self.get_logger().info("Reached center. Waiting 1.5s for joint states to stabilize...")
        time.sleep(1.5)
        self.log_joint_states("AT CENTER")

        current = self.moveit2.compute_fk()
        if current is None:
            self.get_logger().error("FK failed.")
            return

        x = current.pose.position.x
        y = current.pose.position.y
        z = current.pose.position.z
        target_z = z - DESCENT_M

        self.get_logger().info(f"Current z={z:.4f}. Attempting single-shot cartesian descent to z={target_z:.4f} ({DESCENT_M*100:.0f}cm)...")

        time.sleep(0.5)
        target_pose = make_pose(x, y, target_z, *TOP_DOWN)
        result = self.move_to_pose(target_pose, cartesian=True)

        if result:
            self.get_logger().info(f"SUCCESS — descended {DESCENT_M*100:.0f}cm in one shot.")
            self.log_joint_states("AFTER DESCENT")
        else:
            self.get_logger().error(f"FAILED — could not descend {DESCENT_M*100:.0f}cm in one shot.")
            self.log_joint_states("AT FAILURE")

        final = self.moveit2.compute_fk()
        if final:
            self.get_logger().info(f"Final position: x={final.pose.position.x:.4f}, y={final.pose.position.y:.4f}, z={final.pose.position.z:.4f}")


def main(args=None):
    rclpy.init(args=args)
    node = DescentTestNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()