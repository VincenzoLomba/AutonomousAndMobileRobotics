import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vincenzo/Documenti/AMR/ros2pythonvenv/src/lab7workspace/tiagoworkspace/install/launch_pal'
