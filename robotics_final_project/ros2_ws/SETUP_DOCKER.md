# Chessbot Docker Setup

The project uses **two Dockerfiles**, built in sequence. The idea is to keep builds fast: the slow, heavy ROS2 + Kinova layer lives in a base image that rarely changes and only needs to be built once. The project code and Python dependencies live in a second image on top, which rebuilds quickly whenever you change something.

---

## Dockerfile.kortex_jazzy → `kortex_jazzy`

The base image. Contains everything ROS2- and arm-related that rarely changes:

- **ROS2 Jazzy** (`osrf/ros:jazzy-desktop-full`)
- **MoveIt2** (`ros-jazzy-moveit`)
- **Cyclone DDS** set as the default RMW (`rmw_cyclonedds_cpp`) — Kinova's recommendation for MoveIt2
- **`ros2_kortex`** — the Kinova Gen3 Lite driver, cloned from the **`jazzy`** branch and built from source
- A patch to `gen3_lite_macro.xacro` that fixes an upstream bug in the `jazzy` branch (a missing `gripper` parameter on `load_arm`, which otherwise causes `xacro.XacroException: Invalid parameter gripper`)

Each shell launched in this image automatically sources ROS2 and the kortex workspace.

**Build this first** (slow; only rebuild when the ROS2/arm layer changes):

```bash
docker build -f Dockerfile.kortex_jazzy -t kortex_jazzy .
```

---

## Dockerfile.dependencies → `chessbot`

Builds **on top of `kortex_jazzy`** (`FROM kortex_jazzy`) and adds the project layer:

- **Python dependencies:** `ultralytics` (YOLO), `opencv-python`, `numpy`, `stockfish`, `chess`, `setuptools`
- **`pymoveit2`** — the Python MoveIt2 API, cloned and built
- **The project source code** (`src/`), built with `colcon build --symlink-install`

**Build this second** (fast; rebuild whenever you change project code or Python dependencies):

```bash
docker build -f Dockerfile.dependencies -t chessbot .
```

---

## Why two images?

Splitting the build keeps your day-to-day iteration fast:

- **`kortex_jazzy`** holds the heavy, stable layer (ROS2, MoveIt2, the Kinova driver built from source). This takes a long time to build but almost never changes.
- **`chessbot`** holds the fast-moving layer (your code and Python deps). Because it starts `FROM kortex_jazzy`, rebuilding it after a code change only re-runs the cheap top layers — not the whole ROS2/Kinova build.

**Build order matters:** `kortex_jazzy` must exist before you build `chessbot`, since `Dockerfile.dependencies` is based on it.

```bash
# 1. Base image (first time, or when the ROS2/arm layer changes) — slow
docker build -f Dockerfile.kortex_jazzy -t kortex_jazzy .

# 2. Project image (every time you change code or Python deps) — fast
docker build -f Dockerfile.dependencies -t chessbot .
```

---

## Running the Container

Start the `chessbot` container, mounting your live `src/` so code changes are visible without rebuilding, and forwarding the display for the camera/status windows:

```bash
docker run -it --privileged --net=host \
  -v /dev:/dev \
  -v ~/workspaces/ROS2_Robotics_Final_Project/robotics_final_project/ros2_ws/src:/robotics_final_project/ros2_ws/src \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  chessbot
```

### Flags explained

- `--privileged` — gives the container the device access required to talk to the Kinova arm.
- `--net=host` — shares the host network so the container can reach the Kinova arm over Ethernet (at `192.168.1.10`).
- `-v /dev:/dev` — maps the host's USB/serial devices into the container so the Arduino (suction-cup controller) is visible.
- `-v <host src>:/robotics_final_project/ros2_ws/src` — bind-mounts your source code into the container so edits show up live (no rebuild needed for Python changes).
- `-e DISPLAY` and `-v /tmp/.X11-unix:/tmp/.X11-unix` — forward the X display so GUI windows (RViz, the camera/status views) appear on your host.

---

## Launching the Arm (one command)

Inside the container, launch the entire arm stack — **driver + MoveIt + RViz** — with a single command:

```bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10
```

> **Note — this is different from the old Humble setup.** Previously (ROS2 Humble) the driver and MoveIt had to be started in **two separate terminals**. On Jazzy, `robot.launch.py` brings up everything (driver, MoveIt, and RViz) in **one terminal**.

---

## Opening Additional Terminals

The launch command above occupies its terminal. To run nodes, open more shells **inside the already-running container**:

```bash
docker exec -it $(docker ps -q) bash --login
```

Run this once per node you want to start (vision nodes, chess AI, game operation, display, pick-and-place, etc.).

### Flags explained

- `docker exec -it` — runs an interactive shell in the running container (`$(docker ps -q)` resolves to the running container's ID).
- `bash --login` — starts a **login shell**, which sources the ROS2 and workspace setup files (via `.bashrc`) so `ros2` commands and the project packages are on the path. Without `--login`, the environment wouldn't be sourced and `ros2 run ...` would fail.
