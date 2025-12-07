FROM osrf/ros:humble-desktop

# 1. Install System Packages
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-dev \
    build-essential cmake git wget unzip \
    libgtk-3-dev libcanberra-gtk-module libcanberra-gtk3-module \
    libv4l-dev v4l-utils \
&& rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------
# 2. Setup Python Environment and Install Dependencies (Humble Fix)
# Humble uses Python 3.10, which is stable and resolves deep learning conflicts.
# ----------------------------------------------------------------------

# FIX 1: Removed unsupported --break-system-packages flag
RUN pip install --upgrade pip --ignore-installed

# Copy requirements before installing
COPY requirements.txt .

# FIX 2: Removed unsupported --break-system-packages flag
RUN pip install --no-cache-dir -r requirements.txt \
    opencv-python==4.8.0.76 \
    numpy==1.26.4 \
    setuptools==65.5.0 \
    --ignore-installed

# ----------------------------------------------------------------------

# Fix python symlink for system-wide ROS use (now pointing to Python 3.10)
ENV PYTHONPATH="/usr/local/lib/python3.10/dist-packages:${PYTHONPATH}"
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.10 1

# 3. ROS 2 Workspace directory inside container
RUN mkdir -p /robotics_final_project/ros2_ws

# 4. Entry point
WORKDIR /robotics_final_project/ros2_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

