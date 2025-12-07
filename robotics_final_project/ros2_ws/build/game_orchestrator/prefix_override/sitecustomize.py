import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/robotics_final_project/ros2_ws/install/game_orchestrator'
