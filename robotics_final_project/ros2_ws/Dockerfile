FROM osrf/ros:humble-desktop

# -------------------------------------------------------------
# 1. Install System + ROS + MoveIt Dependencies
# -------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-dev \
    build-essential cmake git wget unzip \
    libgtk-3-dev libcanberra-gtk-module libcanberra-gtk3-module \
    libv4l-dev v4l-utils \
    \
    # ROS2 MoveIt packages
    ros-humble-moveit \
    ros-humble-moveit-common \
    ros-humble-moveit-msgs \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-servo \
    \
    # ROS2 control & controllers
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    \
    # ROS message packages required for MoveIt Python API
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-trajectory-msgs \
    \
    # Joint state publishers
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    \
    # Serial communication
    python3-serial \
    && rm -rf /var/lib/apt/lists/*


# -------------------------------------------------------------
# 2. Python Environment Fix
# -------------------------------------------------------------
RUN pip install --upgrade pip --ignore-installed


# -------------------------------------------------------------
# 3. Install Python Packages for YOLO, CV, etc.
# -------------------------------------------------------------
COPY requirements.txt .

RUN pip install --no-cache-dir \
    -r requirements.txt \
    opencv-python==4.8.0.76 \
    numpy==1.26.4 \
    setuptools==65.5.0 \
    --ignore-installed


# -------------------------------------------------------------
# 4. Setup ROS Workspace
# -------------------------------------------------------------
RUN mkdir -p /robotics_final_project/ros2_ws/src
WORKDIR /robotics_final_project/ros2_ws


# -------------------------------------------------------------
# 5. Clone pymoveit2 (REQUIRED FOR python MoveIt API)
# -------------------------------------------------------------
RUN git clone https://github.com/AndrejOrsula/pymoveit2.git src/pymoveit2


# -------------------------------------------------------------
# 6. Build the Workspace (including pymoveit2)
# -------------------------------------------------------------
RUN /bin/bash -c ". /opt/ros/humble/setup.sh && \
    cd /robotics_final_project/ros2_ws && \
    colcon build --symlink-install"


# -------------------------------------------------------------
# 7. Source Workspace + Export Python Path for pymoveit2
# -------------------------------------------------------------
ENV PYTHONPATH="/robotics_final_project/ros2_ws/install/pymoveit2/lib/python3.10/site-packages:${PYTHONPATH}"


# -------------------------------------------------------------
# 8. Entry Point
# -------------------------------------------------------------
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
