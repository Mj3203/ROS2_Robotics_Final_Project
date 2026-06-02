FROM chessbot_base
 
# -------------------------------------------------------------
# 1. Python Environment Fix
# -------------------------------------------------------------
RUN pip install --upgrade pip --ignore-installed
 
 
# -------------------------------------------------------------
# 2. Install Python Packages for YOLO, CV, etc.
# -------------------------------------------------------------
RUN pip install --no-cache-dir \
    ultralytics \
    opencv-python==4.8.0.76 \
    numpy==1.26.4 \
    setuptools==65.5.0 \
    stockfish \
    chess \
    --ignore-installed
 
 
# -------------------------------------------------------------
# 3. Clone & Build pymoveit2 (REQUIRED FOR python MoveIt API)
# -------------------------------------------------------------
RUN mkdir -p /robotics_final_project/ros2_ws/src
WORKDIR /robotics_final_project/ros2_ws
 
RUN git clone https://github.com/AndrejOrsula/pymoveit2.git src/pymoveit2
 
RUN /bin/bash -c ". /opt/ros/humble/setup.sh && \
    cd /robotics_final_project/ros2_ws && \
    colcon build --symlink-install"
 
# -------------------------------------------------------------
# 4. Export Python Path for pymoveit2
# -------------------------------------------------------------
ENV PYTHONPATH="/robotics_final_project/ros2_ws/install/pymoveit2/lib/python3.10/site-packages"
 
 
# -------------------------------------------------------------
# 5. Copy Project Source Code & Build Workspace
# -------------------------------------------------------------
COPY src/ /robotics_final_project/ros2_ws/src/
 
RUN /bin/bash -c ". /opt/ros/humble/setup.sh && \
    . /robotics_final_project/ros2_ws/install/setup.sh && \
    cd /robotics_final_project/ros2_ws && \
    colcon build --symlink-install"
 
 
# -------------------------------------------------------------
# 6. Entry Point
# -------------------------------------------------------------
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
