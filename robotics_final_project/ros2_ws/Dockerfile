FROM chessbot_base

# -------------------------------------------------------------
# 1. Install Python Packages for YOLO, CV, etc.
# -------------------------------------------------------------
RUN pip install --no-cache-dir \
    ultralytics \
    opencv-python==4.8.0.76 \
    numpy==1.26.4 \
    setuptools==65.5.0 \
    stockfish \
    chess \
    --break-system-packages

# -------------------------------------------------------------
# 2. Clone & Build pymoveit2 (REQUIRED FOR python MoveIt API)
# -------------------------------------------------------------
RUN mkdir -p /robotics_final_project/ros2_ws/src
WORKDIR /robotics_final_project/ros2_ws

RUN git clone https://github.com/AndrejOrsula/pymoveit2.git src/pymoveit2

RUN /bin/bash -c ". /opt/ros/jazzy/setup.sh && \
    cd /robotics_final_project/ros2_ws && \
    colcon build --packages-select pymoveit2"

# -------------------------------------------------------------
# 3. Export Python Path for pymoveit2
# -------------------------------------------------------------
ENV PYTHONPATH="/robotics_final_project/ros2_ws/install/pymoveit2/lib/python3.12/site-packages"

# -------------------------------------------------------------
# 4. Copy Project Source Code & Build Workspace
# -------------------------------------------------------------
COPY src/ /robotics_final_project/ros2_ws/src/

RUN /bin/bash -c ". /opt/ros/jazzy/setup.sh && \
    . /kortex_ws/install/setup.sh && \
    . /robotics_final_project/ros2_ws/install/setup.sh && \
    cd /robotics_final_project/ros2_ws && \
    colcon build --symlink-install --packages-ignore pymoveit2"

# -------------------------------------------------------------
# 5. Entry Point
# -------------------------------------------------------------
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]