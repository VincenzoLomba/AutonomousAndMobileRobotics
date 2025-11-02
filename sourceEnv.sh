
# Activating the Python environment IFF not already actived
if [[ -z "$VIRTUAL_ENV" ]]; then
    source /home/vincenzo/Documenti/AMR/ros2pythonvenv/bin/activate
fi
export LIBGL_ALWAYS_INDIRECT=0                      # Setting up the current terminal (in the current PyEnv) to rely on the GPU w.r.t. the CPU (for example when using Gazebo or RVIDZ) (1/3)
export __NV_PRIME_RENDER_OFFLOAD=1                  # Setting up the current terminal (in the current PyEnv) to rely on the GPU w.r.t. the CPU (for example when using Gazebo or RVIDZ) (2/3)
export __GLX_VENDOR_LIBRARY_NAME=nvidia             # Setting up the current terminal (in the current PyEnv) to rely on the GPU w.r.t. the CPU (for example when using Gazebo or RVIDZ) (3/3)
source /opt/ros/humble/setup.bash                   # Sourching ROS2 commands (activating ROS2 in the current environemnt)
source /usr/share/gazebo/setup.sh                   # Sourching Gazebo commands FOR ROS2 (enable Gazebo usage for ROS2)
cd /home/vincenzo/Documenti/AMR/ros2pythonvenv/src  # Moving to the workspaces folder
export TURTLEBOT3_MODEL=burger                      # In case of using TurtleBOT3, already setting the related to-be-used model as "burger"
export ROS_OS_OVERRIDE=ubuntu:jammy                 # Telling to ROS2 that the name of the current OS muast be considered as ubuntu:jammy (and NOT Mint)
