import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vincenzo/Documenti/AMR/ros2pythonvenv/src/lab4workspace/install/turtlebot3_teleop'
