import time
import subprocess
from pick_and_place_pkg.constants import (
    SQUARE_COORDS_M, BOARD_Z, HOVER_ABOVE_BOARD_M,
    CENTER_HOVER_ABOVE_BOARD_M, PLACE_Z_OFFSET, TOP_DOWN,
    make_pose, square_center_in_world, compute_board_center
)

def wait_for_moveit(node, timeout_s=60):
    node.get_logger().info(f"Waiting up to {timeout_s}s for planner and joint states...")

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
        node.get_logger().warn("Timed out waiting for planner service.")

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
        node.get_logger().warn("Timed out waiting for joint states.")

    ready = planner_ready and joints_ready
    if ready:
        node.get_logger().info("Planner and joint states ready.")

    return ready


def lock_gripper(node):
    if node.gripper is None:
        node.get_logger().warn("Cannot lock gripper: gripper interface is missing.")
        return

    def blocked_open(*args, **kwargs):
        node.get_logger().warn("Gripper open blocked — gripper is locked.")
        return False

    try:
        if not hasattr(node.gripper, "original_open"):
            node.gripper.original_open = getattr(node.gripper, "open", None)
        node.gripper.open = blocked_open
        node.get_logger().info("Gripper locked successfully.")
    except Exception as e:
        node.get_logger().warn(f"Failed to lock gripper: {e}")


def task_tune(node):
    ready = wait_for_moveit(node, timeout_s=60)
    if not ready:
        node.get_logger().warn("Continuing tuning despite incomplete readiness.")

    tune_mode = node.get_parameter("tune_mode").get_parameter_value().string_value.lower()

    if tune_mode == "home":
        node.get_logger().info("[TUNE] Moving to home position.")
        if not node.move_to_joints(node.j_home):
            node.get_logger().error("[TUNE] Failed to move to home position.")
        return

    if tune_mode == "hold":
        node.get_logger().info("[TUNE] Closing and locking gripper.")
        node.close_gripper()
        lock_gripper(node)
        node.get_logger().info("[TUNE] Gripper is now locked.")
        return

    if tune_mode == "open":
        node.get_logger().info("[TUNE] Opening gripper.")
        node.open_gripper()
        return

    center_x, center_y, center_z = compute_board_center()
    center_hover_z = center_z + CENTER_HOVER_ABOVE_BOARD_M
    center_hover_pose = make_pose(center_x, center_y, center_