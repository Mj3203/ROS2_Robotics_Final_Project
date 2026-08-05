# Setup Commands


## Running the System

### Terminal 1 — Launch the arm

Launch the full arm stack (driver + MoveIt + RViz) with a single command:

```bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10
```

### Terminal 2 — Raw camera feed: computer_vision_pkg raw_camera_feed

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run computer_vision_pkg raw_camera_feed
```

### Terminal 3 — Homography transform: computer_vision_pkg homography_transform

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run computer_vision_pkg homography_transform
```

### Terminal 4 — Scan and detect: computer_vision_pkg scan_and_detect

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run computer_vision_pkg scan_and_detect
```

### Terminal 5 — Chess AI: chess_ai_pkg chess_ai

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run chess_ai_pkg chess_ai
```

### Terminal 6 — Arduino bridge: arduino_pkg arduino_node

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run arduino_pkg arduino_node
```

### Terminal 7 — Pick and place: pick_and_place_pkg pick_and_place

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run pick_and_place_pkg pick_and_place
```

`pick_and_place` defaults to **serve** mode, where it waits for `MoveRobot` requests from the game loop. It also supports a `home` task and a set of tune modes (below).

### Terminal 8 — Game operation: game_operation_pkg game_operation

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run game_operation_pkg game_operation
```

### Terminal 9 — Display output: display_output_pkg display_output

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run display_output_pkg display_output
```

---

## Tune Commands

Tune modes drive the arm to specific poses for calibration and testing, without running a full game. They use the `task:=tune` parameter plus a `tune_mode` (and a `tune_square` for the square-based modes).

```bash
# Move to the home pose
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=home

# Move to the center hover pose above the board
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=center

# Hold position (keeps current pose)
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=hold

# Open the gripper / release suction
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=open

# Hover over a square, then descend to board level (e.g. e4)
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=square -p tune_square:=e4

# Hover over a square, descend, then lift back up (e.g. e4)
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=lift -p tune_square:=e4

# Move directly to a square's hover (no center-hover waypoint, e.g. h8)
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=direct -p tune_square:=h8
```

The `square`, `lift`, and `direct` modes require a `tune_square` (e.g. `a1`–`h8`). The `home`, `center`, `hold`, and `open` modes do not.

### Snake test

Visits every square on the board in sequence — useful for checking reachability across the whole board:

```bash
ros2 run pick_and_place_pkg snake_test
```
