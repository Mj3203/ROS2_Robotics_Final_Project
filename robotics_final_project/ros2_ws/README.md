# Chessbot Docker Setup

The project uses two Dockerfiles to keep builds fast. The idea is that the slow,
heavy dependencies are in the base image and only need to be built once. Your project
code and Python packages live in the main image, which rebuilds quickly whenever you
make changes.

---

## Dockerfile.base → `chessbot_base`

Contains everything ROS2-related that rarely changes:

- ROS2 Humble (via `osrf/ros:humble-desktop`)
- MoveIt2 and all ros-humble packages
- ROS2 Control and controllers
- Joint state publishers
- Serial communication (pyserial)

**Build once, or only when you add/change a ROS2 package:**
```bash
docker build -f Dockerfile.base -t chessbot_base .
```

---

## Dockerfile → `chessbot`

Builds on top of `chessbot_base` and contains everything else:

- Python environment setup
- OpenCV, NumPy, Ultralytics (YOLO)
- pymoveit2 (Python MoveIt2 API)
- Your project source code (`chess_ai_pkg`, `chess_vision_pkg`, `pick_and_place_pkg`, etc.)
- Workspace colcon build

**Build whenever you change project code or Python dependencies:**
```bash
docker build -t chessbot .
```

---

## Running the Container

```bash
docker run -it --privileged --net=host -v /dev:/dev chessbot
```

- `--privileged` — required for Kinova arm access
- `--net=host` — required for Kinova ethernet connection
- `-v /dev:/dev` — maps USB/serial devices so Arduino is visible

---

## Typical Workflow

```bash
# 1. Build the base (first time only, ~20-30 mins)
docker build -f Dockerfile.base -t chessbot_base .

# 2. Build the app (every time you change code, ~5-10 mins)
docker build -t chessbot .

# 3. Run
docker run -it --privileged --net=host -v /dev:/dev chessbot
```
