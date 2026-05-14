# ROS2 Chess Robot

A fully autonomous chess-playing robot system built with ROS2 Humble. The system uses a Kinova Gen3 Lite robotic arm to physically move chess pieces, a YOLO-based vision pipeline to detect board state, and Stockfish to select the best move.

---

## System Overview

```
Camera
  └─► raw_camera_feed_node ──/raw_camera_feed──► homography_transform_node
                                                          │
                                               /processed_camera_feed
                                                          │
                                                 scan_board_node ◄── get_board_state (service)
                                                          │
                                               /live_detection_feed
                                               /scan_result_feed
                                                          │
                                               game_operation_node
                                                ├── get_board_state (client)
                                                ├── get_best_move (client)
                                                ├── move_robot (client)
                                                └── /ai_move (publisher)
                                                          │
                                                 chess_ai_node ◄── get_best_move (service)
                                                          │
                                               pick_and_place_node ◄── move_robot (service)
                                                          │
                                                  Kinova Gen3 Lite
                                                  Arduino + Solenoid
```

---

## Packages

| Package | Node | Description |
|---|---|---|
| `computer_vision_pkg` | `raw_camera_feed_node` | Captures raw camera frames and publishes to `/raw_camera_feed` |
| `computer_vision_pkg` | `homography_transform_node` | Detects ArUco markers and warps board to top-down view |
| `piece_detection_pkg` | `scan_board_node` | Runs YOLO to detect pieces and map them to chess squares |
| `game_operation_pkg` | `game_operation_node` | Orchestrates the full game loop |
| `chess_ai_pkg` | `chess_ai_node` | Runs Stockfish at depth 23 to select best move |
| `pick_and_place_pkg` | `pick_and_place_node` | Controls the Kinova arm and Arduino solenoid |
| `display_output_pkg` | `display_output_node` | 2x2 display window showing all camera feeds |

---

## ROS2 Topics

| Topic | Publisher | Subscriber | Description |
|---|---|---|---|
| `/raw_camera_feed` | `raw_camera_feed_node` | `homography_transform_node`, `display_output_node` | Raw 1920x1080 camera frames |
| `/processed_camera_feed` | `homography_transform_node` | `scan_board_node`, `display_output_node` | Warped top-down board image |
| `/live_detection_feed` | `scan_board_node` | `display_output_node` | Continuous YOLO annotated frames |
| `/scan_result_feed` | `scan_board_node` | `display_output_node` | Snapshot from last board scan |
| `/button_feed` | `pick_and_place_node` | *(not yet connected)* | Arduino button press events |

## ROS2 Services

| Service | Server | Client | Description |
|---|---|---|---|
| `get_board_state` | `scan_board_node` | `game_operation_node` | Returns board state as JSON dict |
| `get_best_move` | `chess_ai_node` | `game_operation_node` | Returns best UCI move from Stockfish |
| `move_robot` | `pick_and_place_node` | `game_operation_node` | Commands arm to execute a UCI move |

---

## Game Loop

1. Robot (white) requests opening move from Stockfish via `get_best_move`
2. Arm executes the move via `move_robot`
3. Scan 1 — board state captured after robot moves
4. Human (black) makes their move
5. Human presses button (or ENTER) to trigger Scan 2
6. `game_operation_node` diffs Scan 1 vs Scan 2 to detect human's move
7. Human's move sent to Stockfish via `get_best_move`
8. Repeat from step 2

---

## Hardware

- **Robotic Arm:** Kinova Gen3 Lite (IP: 192.168.1.10)
- **Gripper:** Arduino-controlled solenoid
- **Camera:** USB camera at `/dev/video0` (1920x1080)
- **Board Markers:** 4x ArUco markers (DICT_5X5_250, IDs 0-3) at board corners

---

## Docker Setup

The project runs inside Docker with ROS2 Humble, MoveIt2, and ros2_kortex pre-installed.

**Build the images:**
```bash
docker build -f Dockerfile.base -t chessbot_base .
docker build -f Dockerfile -t chessbot .
```

**Run the container:**
```bash
xhost +local:docker

docker run -it --privileged --net=host \
  -v /dev:/dev \
  -v ~/workspaces/ROS2_Robotics_Final_Project/robotics_final_project/ros2_ws/src:/robotics_final_project/ros2_ws/src \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  chessbot
```

---

## Running the System

**Terminal 1 — Kinova driver:**
```bash
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10
```

**Terminal 2 — MoveIt2:**
```bash
source /opt/ros/humble/setup.bash
source /kortex_ws/install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10
```

**Terminal 3 — Raw camera feed:**
```bash
source /opt/ros/humble/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run computer_vision_pkg raw_camera_feed
```

**Terminal 4 — Homography transform:**
```bash
source /opt/ros/humble/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run computer_vision_pkg homography_transform
```

**Terminal 5 — Piece detection:**
```bash
source /opt/ros/humble/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run piece_detection_pkg scan_board
```

**Terminal 6 — Chess AI:**
```bash
source /opt/ros/humble/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run chess_ai_pkg chess_ai
```

**Terminal 7 — Pick and place:**
```bash
source /opt/ros/humble/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run pick_and_place_pkg pick_and_place
```

**Terminal 8 — Game operation:**
```bash
source /opt/ros/humble/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run game_operation_pkg game_operation
```

**Terminal 9 — Display output:**
```bash
source /opt/ros/humble/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run display_output_pkg display_output
```

---

## Dependencies

- ROS2 Humble
- MoveIt2
- ros2_kortex (pinned to commit `3ca0e71`)
- Python: `python-chess`, `ultralytics`, `pymoveit2`, `pyserial`, `stockfish`
