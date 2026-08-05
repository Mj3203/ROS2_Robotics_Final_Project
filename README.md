# Project Overview

Created an autonomous chess-playing robot that uses a Kinova Gen3 Lite robot arm, RGB camera, and a custom compressed air end effector programmed by an Arduino Nano.

---
## How It Works

1. **Vision** — A camera captures the board. ArUco tags at the corners of the chess board allows the homography node warp the image into a top-down view, correcting for any angle/position.
2. **Object Detection** — A custom trained YOLO model scans the feed and maps every detected piece to a board square, producing a JSON representation of the current board state.
3. **Chess AI** — The scanned board state is compared against legal chess moves to determine what the human just played. Stockfish receives the validated move and calculates the best response.
5. **Pick and Place** — The robot arm uses MoveIt2 to plans and executes the it's moves.
6. **Game Operation** — A central node loops the steps listed above in a specified order, depending on which color the player chooses to play.
7. **Display Output** — Camera feeds, game status, and move history are shown on a physical display.

Each portion is independent, so each package/node can be debugged, restarted, or replaced independently of one another.

| Package | Responsibility |
|---|---|
| `computer_vision_pkg` | Captures the raw camera feed, warps it into a flat top-down view using ArUco corner markers, then runs YOLO to detect each piece and report the board state (also serves on-demand board scans). |
| `chess_ai_pkg` | Owns the authoritative chess board. Validates human moves and uses Stockfish to choose the robot's moves. |
| `pick_and_place_pkg` | Drives the Kinova arm and suction cup to physically move and capture pieces. |
| `game_operation_pkg` | The conductor. Runs the turn-by-turn game loop and calls the other packages in the right order. |
| `display_output_pkg` | Shows a 2×2 grid of camera/detection views plus a status bar and game log. |
| `custom_interface` | Defines the custom service messages the packages use to talk to each other. |

### How they communicate

**Topics (continuous streams):**

- `raw_camera_feed` — live camera image (published by `computer_vision_pkg`)
- `processed_camera_feed` — straightened top-down board view (published by `computer_vision_pkg`)
- `live_detection_feed` — board view with YOLO detections drawn on (published by `computer_vision_pkg`)
- `game_status_feed` — current game state and move info, used by the display (published by `game_operation_pkg`)

**Services (request/response, defined in `custom_interface`):**

- `ScanBoard` — "scan the board and tell me what pieces are where" → served by `computer_vision_pkg`
- `ValidateMove` — "is the scanned board a legal move?" → served by `chess_ai_pkg`
- `GetBestMove` — "here's the human's move, what's the robot's reply?" → served by `chess_ai_pkg`
- `MoveRobot` — "physically make this move (and capture if needed)" → served by `pick_and_place_pkg`

---

## Game Loop

At startup, the operator is asked whether the robot plays **White** or **Black**. From there the game runs automatically, one turn at a time.

### If the robot plays White (moves first)

1. The robot immediately asks the chess engine for its opening move.
2. The arm executes that move on the board.
3. The robot then waits for the human to respond.

### If the robot plays Black (waits for the human)

1. The robot waits for the human to make the first move.
2. The human moves a piece, then presses **ENTER** to signal "your turn."

### Each turn, from the robot's perspective

1. **Wait for the human.** The human makes their move physically and presses **ENTER**.
2. **Scan the board.** `game_operation_pkg` calls `ScanBoard`; `computer_vision_pkg` runs YOLO and returns the current piece layout.
3. **Validate the move.** `game_operation_pkg` calls `ValidateMove`; `chess_ai_pkg` compares the scan against the legal moves from the current position.
   - If the scan doesn't match any legal move, the move is rejected with a reason and the human is asked to redo it (back to step 1).
4. **Ask the AI for a reply.** Once the human's move is valid, `game_operation_pkg` calls `GetBestMove`. `chess_ai_pkg` applies the human's move, runs Stockfish, and returns the robot's best move along with whether it is a **capture** and the resulting **game status** (e.g. check, checkmate, stalemate, draw).
5. **Check for game over.** If the status is checkmate/stalemate/draw, the loop ends and the result is reported.
6. **Execute the robot's move.** Otherwise `game_operation_pkg` calls `MoveRobot` with the move and the capture flag; `pick_and_place_pkg` drives the arm:
   - **Normal move:** hover over the source square → descend → suction on → lift → travel to the target square → descend → suction off → retract.
   - **Capture handling:** if the move is a capture, the arm first picks up the enemy piece on the target square and drops it into a side basket, **then** moves its own piece onto that square.
7. **Back to waiting.** Control returns to step 1 for the human's next move.

Throughout, `game_operation_pkg` publishes the current state to `game_status_feed`, so `display_output_pkg` always shows the latest move, status, and log.

---

## Docker Setup

The project uses **two Dockerfiles**, built in sequence. The first builds the heavy ROS2 + Kinova base and rarely changes; the second adds the Python dependencies and the project code on top, and rebuilds quickly.

### 1. `Dockerfile.kortex_jazzy` → `kortex_jazzy` image

The base image. Contains everything ROS2- and arm-related:

- **ROS2 Jazzy** (`osrf/ros:jazzy-desktop-full`)
- **MoveIt2** (`ros-jazzy-moveit`)
- **Cyclone DDS** as the default RMW (Kinova's recommendation for MoveIt2)
- **`ros2_kortex`** — the Kinova Gen3 Lite driver, cloned from the `jazzy` branch and built from source
- A patch to `gen3_lite_macro.xacro` that fixes an upstream bug in the `jazzy` branch (a missing `gripper` parameter on `load_arm` that otherwise causes a xacro error)

Build it first (slow, only needs rebuilding when the ROS/arm layer changes):

```bash
docker build -f Dockerfile.kortex_jazzy -t kortex_jazzy .
```

### 2. `Dockerfile.dependencies` → `chessbot` image

Builds **on top of `kortex_jazzy`** and adds the project layer:

- **Python dependencies:** `ultralytics` (YOLO), `opencv-python`, `numpy`, `stockfish`, `chess`, `setuptools`
- **`pymoveit2`** — the Python MoveIt2 API, cloned and built
- **The project source code** (`src/`), built with `colcon build --symlink-install`

Build it whenever you change project code or Python dependencies (fast):

```bash
docker build -f Dockerfile.dependencies -t chessbot .
```

> Build order matters: `kortex_jazzy` must exist before building `chessbot`, since `Dockerfile.dependencies` starts `FROM kortex_jazzy`.

---

## Running the System

### Terminal 1 — start the container and launch the arm

Start the `chessbot` container, mounting your live `src/` so code changes are visible without rebuilding, and forwarding the display for the camera/status windows:

```bash
docker run -it --privileged --net=host \
  -v /dev:/dev \
  -v ~/workspaces/ROS2_Robotics_Final_Project/robotics_final_project/ros2_ws/src:/robotics_final_project/ros2_ws/src \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  chessbot
```

- `--privileged` — required for Kinova arm access
- `--net=host` — required for the Kinova ethernet connection
- `-v /dev:/dev` — maps USB/serial devices so the Arduino is visible
- `-e DISPLAY` / `-v /tmp/.X11-unix` — forwards the GUI windows

Inside the container, launch the full arm stack (driver + MoveIt + RViz) with a single command:

```bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10
```

### Additional terminals — open more shells in the running container

Each node runs in its own shell. To open another shell inside the already-running container:

```bash
docker exec -it $(docker ps -q) bash --login
```

Run this once per node you want to start.

### Running each node

```bash
# Vision pipeline
ros2 run computer_vision_pkg raw_camera_feed
ros2 run computer_vision_pkg homography_transform
ros2 run computer_vision_pkg scan_and_detect

# Chess engine and game coordinator
ros2 run chess_ai_pkg chess_ai
ros2 run game_operation_pkg game_operation

# Display
ros2 run display_output_pkg display_output

# Arm — serve mode (waits for MoveRobot requests during a game)
ros2 run pick_and_place_pkg pick_and_place
```

`pick_and_place` defaults to **serve** mode, where it waits for `MoveRobot` requests from the game loop. It also supports a `home` task and a set of tune modes (below).

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
