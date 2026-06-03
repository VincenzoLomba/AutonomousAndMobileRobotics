#!/bin/bash

# ==============================================================================
# |     A simple script to BUILD ALL what is related to the usage of Tiago     |
# |             (note: this file makes use of the fastROS Library)             |
# | (you can call this file from the terminal issuing "source buildTiago.sh")  |
# ==============================================================================

# Save current directory
CURRENT_DIR=$(pwd)
echo ""
fastrosEcho INFO "Starting Tiago building & setup process..."

# -------- Correctly sourcing ROS2 with FastROS --------
fastrosEcho INFO "First of all, sourcing ROS2 Humble..."
rossource
echo ""

# -------- ROS2 Workspace --------
fastrosEcho INFO "Entering 'ros2_ws' folder (need of the ros-controls family, also containing Tiago controllers installation)..."
cd ros2_ws/ || { fastrosEcho ERROR "Folder 'ros2_ws' not found!"; return 1; }

fastrosEcho INFO "Executing colcon build in 'ros2_ws'..."
echo ""
colcon build || { fastrosEcho ERROR "Error during 'ros2_ws' building!"; return 1; }

echo ""
fastrosEcho INFO "Sourcing ros2_ws/install/setup.bash..."
source install/setup.bash

# Back to the starting directory
cd "$CURRENT_DIR" || return 1

# -------- Tiago Workspace --------
fastrosEcho INFO "Entering 'tiago_ws' folder..."
cd tiago_ws/ || { fastrosEcho ERROR "Folder 'tiago_ws' not found!"; return 1; }

fastrosEcho INFO "Executing colcon build in 'tiago_ws'..."
echo ""
colcon build || { fastrosEcho ERROR "Error during 'tiago_ws' building!"; return 1; }

echo ""
fastrosEcho INFO "Sourcing tiago_ws/install/setup.bash..."
source install/setup.bash

# Back to the starting directory
cd "$CURRENT_DIR" || return 1

# -------- Done --------
fastrosEcho INFO "All done ✅ You're now ready to play with Tiago!"
fastrosEcho LINK "If you want to get access to Tiago camera, issue command: " "ros2 run rqt_image_view rqt_image_view"
fastrosEcho LINK "If you want to manually regulate all Tiago joints, issue command: " "ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller"
fastrosEcho LINK "If you want to start MoveIt! for Tiago on RViz, issue command: " "ros2 launch tiago_moveit_config moveit_rviz.launch.py"
echo ""

