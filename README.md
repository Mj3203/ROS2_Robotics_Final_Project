# ROS2 Chess Robot

A fully autonomous chess-playing robot system built with ROS2 Jazzy. The system uses a Kinova Gen3 Lite robotic arm to physically move chess pieces, a YOLO-based vision pipeline to detect board state, and Stockfish to select the best move.

---

## System Overview

```
Camera
  └─► raw_camera_feed_node ──/raw_camera_feed──► homography_transform_node
                                                          │
                                               /processed_camera_feed
                                                          │
                                                 scan_and_detect_node ◄── scan_board (service)
                                                          │
                                               /live_detection_feed
                                                          │
                                               game_operation_node
                                                ├── scan_board (client)
                                                ├── validate_move (client)
                                                ├── get_best_move (client)
                                                ├── move_robot (client)
                                                ├── /game_status_feed (publisher)
                                                └── /move_complete (subscriber)
                                                          │
                                                 chess_ai_server ◄── validate_move (service)
                                                                  ◄── get_best_move (service)
                                                          │
                                     pick_and_place_pkg
                                       ├── move_robot_service_node ◄── move_robot (service)
                                       └── pick_and_place_node ──► /move_complete
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
| `computer_vision_pkg` | `scan_and_detect_node` | Runs YOLO to detect pieces and map them to chess squares |
| `game_operation_pkg` | `game_operation_node` | Orchestrates the full game loop |
| `chess_ai_pkg` | `chess_ai_server` | Runs Stockfish at depth 10 to select best move |
| `pick_and_place_pkg` | `pick_and_place_node` | Controls the Kinova arm and Arduino solenoid |
| `pick_and_place_pkg` | `move_robot_service_node` | Serves `move_robot` and queues moves for the arm |
| `display_output_pkg` | `display_output_node` | 2x2 display window showing all camera feeds |
| `custom_interface` | *(interfaces only)* | Defines the `ScanBoard`, `ValidateMove`, `GetBestMove`, and `MoveRobot` service messages |

---

## ROS2 Topics

| Topic | Publisher | Subscriber | Description |
|---|---|---|---|
| `/raw_camera_feed` | `raw_camera_feed_node` | `homography_transform_node`, `display_output_node` | Raw 1920x1080 camera frames |
| `/processed_camera_feed` | `homography_transform_node` | `scan_and_detect_node`, `display_output_node` | Warped top-down board image |
| `/live_detection_feed` | `scan_and_detect_node` | `display_output_node` | Continuous YOLO annotated frames |
| `/game_status_feed` | `game_operation_node` | `display_output_node` | Current game state and move info (JSON) |
| `/move_complete` | `pick_and_place_node` | `game_operation_node` | Signals the arm finished a move (`SUCCESS`/`ERROR`) |

## ROS2 Services

| Service | Server | Client | Description |
|---|---|---|---|
| `scan_board` | `scan_and_detect_node` | `game_operation_node` | Scans the board and returns its state as JSON |
| `validate_move` | `chess_ai_server` | `game_operation_node` | Checks the scanned board against the legal moves |
| `get_best_move` | `chess_ai_server` | `game_operation_node` | Returns best UCI move from Stockfish |
| `move_robot` | `move_robot_service_node` | `game_operation_node` | Commands arm to execute a UCI move |

---

## Game Loop

1. At startup the operator chooses whether the robot plays **White** or **Black**.
2. If the robot is White, it requests its opening move via `get_best_move` and the arm executes it via `move_robot`.
3. The human makes their move physically, then presses **ENTER**.
4. `game_operation_node` calls `scan_board`; `scan_and_detect_node` runs YOLO and returns the current board state.
5. `game_operation_node` calls `validate_move`; `chess_ai_server` compares the scan against the legal moves. If it matches none, the move is rejected and the human retries (back to step 3).
6. The validated human move is sent to `get_best_move`; `chess_ai_server` applies it, runs Stockfish, and returns the robot's reply with a capture flag and game status.
7. If the status is checkmate/stalemate/draw, the game ends. Otherwise the arm executes the reply via `move_robot` (capturing the enemy piece first if needed) and reports completion on `/move_complete`.
8. Repeat from step 3.

---

## Hardware

- **Robotic Arm:** Kinova Gen3 Lite (IP: 192.168.1.10)
- **Gripper:** Arduino-controlled solenoid
- **Camera:** USB camera at `/dev/video0` (1920x1080)
- **Board Markers:** 4x ArUco markers (DICT_5X5_250, IDs 0-3) at board corners

---

## Docker Setup

The project runs inside Docker with ROS2 Jazzy, MoveIt2, and ros2_kortex pre-installed.

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

**Terminal 1 — Kinova driver + MoveIt2:**
```bash
source /opt/ros/jazzy/setup.bash
source /kortex_ws/install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10
```

**Terminal 2 — Raw camera feed:**
```bash
source /opt/ros/jazzy/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run computer_vision_pkg raw_camera_feed
```

**Terminal 3 — Homography transform:**
```bash
source /opt/ros/jazzy/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run computer_vision_pkg homography_transform
```

**Terminal 4 — Piece detection:**
```bash
source /opt/ros/jazzy/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run computer_vision_pkg scan_and_detect
```

**Terminal 5 — Chess AI:**
```bash
source /opt/ros/jazzy/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run chess_ai_pkg chess_ai
```

**Terminal 6 — Pick and place:**
```bash
source /opt/ros/jazzy/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run pick_and_place_pkg pick_and_place
```

**Terminal 7 — Game operation:**
```bash
source /opt/ros/jazzy/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run game_operation_pkg game_operation
```

**Terminal 8 — Display output:**
```bash
source /opt/ros/jazzy/setup.bash
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 run display_output_pkg display_output
```

---

## Dependencies

- ROS2 Jazzy
- MoveIt2
- ros2_kortex (pinned to commit `3ca0e71`)
- Python: `python-chess`, `ultralytics`, `pymoveit2`, `pyserial`, `stockfish`
