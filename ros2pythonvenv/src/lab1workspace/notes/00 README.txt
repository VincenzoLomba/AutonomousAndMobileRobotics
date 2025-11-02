
(1) A problem that can arise using rosdep in Mint:

(ros2pythonvenv) vincenzo@VinsTUFPC:~/Documenti/AMR/ros2pythonvenv/src/lab1workspace$ rosdep install -i --from-path src --rosdistro humble
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
turtlesim: Unsupported OS [mint]
(ros2pythonvenv) vincenzo@VinsTUFPC:~/Documenti/AMR/ros2pythonvenv/src/lab1workspace$ export ROS_OS_OVERRIDE=ubuntu:jammy

(2) A problem that can arise using rosdep in Mint, manually install unlocated packages cause the usage of Mint:

sudo apt install qtbase5-dev qtbase5-dev-tools libqt5core5a libqt5gui5 libqt5widgets5 

(3) A problem that can arise we trying to use same ROS2 related packages BUT within a certain python env (install them in it"):

# Attiva il virtualenv
source ~/Documenti/AMR/ros2pythonvenv/bin/activate

# Installa le librerie Python di ROS necessarie
pip install empy==3.3.4
pip install catkin_pkg
pip install lark-parser==0.11.3
