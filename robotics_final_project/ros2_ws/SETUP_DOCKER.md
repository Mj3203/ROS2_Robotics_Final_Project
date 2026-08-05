# Docker Setup

The project uses **two Dockerfiles**, built in sequence. The idea is that most of the changes will come from the Python dependencies. Thus rather than rebuilding the whole container, rebuild a single layer.
- **Layer 1** ROS2 + Kinova
- **Layer 2** The project code and Python dependencies

---

## Dockerfile.kortex_jazzy

The base image. Contains everything ROS2 and Kinvoa related.

**Build this first**

```bash
docker build -f Dockerfile.kortex_jazzy -t kortex_jazzy .
```

---

## Dockerfile.dependencies

Builds **on top of `kortex_jazzy`** 

**Build this second**

```bash
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
---

## Opening Additional Terminals

The launch command above occupies its terminal. To open more shells:

```bash
docker exec -it $(docker ps -q) bash --login
```

Run this per terminal when you want to run a node.

