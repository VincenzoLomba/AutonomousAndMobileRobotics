#!/bin/bash
# ====================================================
# Script to build and prepare the workspace for Tiago
# Usage: source nomefile.sh
# ====================================================

# Save current directory
CURRENT_DIR=$(pwd)
echo ""
fastrosEcho INFO "Starting Tiago build and setup process..."

# -------- ROS2 Workspace --------
fastrosEcho INFO "Entering ros2workspace (for Tiago controllers installation)..."
cd ros2workspace/ || { fastrosEcho ERROR "Folder 'ros2workspace' not found!"; return 1; }

fastrosEcho INFO "Executing colcon build in ros2workspace..."
echo ""
colcon build || { fastrosEcho ERROR "Error during ros2workspace building!"; return 1; }

echo ""
fastrosEcho INFO "Sourcing install/setup.bash (ros2workspace)..."
source install/setup.bash

# Back to the starting directory
cd "$CURRENT_DIR" || return 1

# -------- Tiago Workspace --------
fastrosEcho INFO "Entering tiagoworkspace..."
cd tiagoworkspace/ || { fastrosEcho ERROR "Folder 'tiagoworkspace' not found!"; return 1; }

fastrosEcho INFO "Executing colcon build in tiagoworkspace..."
echo ""
colcon build || { fastrosEcho ERROR "Error during tiagoworkspace building!"; return 1; }

echo ""
fastrosEcho INFO "Sourcing install/setup.bash (tiagoworkspace)..."
source install/setup.bash

# Back to the starting directory
cd "$CURRENT_DIR" || return 1

# -------- Done --------
fastrosEcho INFO "All done ✅ You're now ready to play with Tiago!"
echo ""

# -------- Final instructions --------
fastrosEcho LINK "Launch Tiago Gazebo simulation: " "ros2 launch tiago_gazebo tiago_gazebo.launch.py"
fastrosEcho LINK "Launch Tiago control panel:     " "ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller"
fastrosEcho LINK "Start MoveIt! on RViz:          " "ros2 launch tiago_moveit_config moveit_rviz.launch.py"
echo ""

