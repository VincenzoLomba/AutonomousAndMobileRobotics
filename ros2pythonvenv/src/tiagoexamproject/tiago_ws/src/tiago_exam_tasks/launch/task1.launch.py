
# This is the launch file for Project Task1: autonomous exploration and mapping

# Necessary imports
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Using "DeclareLaunchArgument" to declare a launch argument/parameter (that can be set from the command line when launching the file)
    # This is the name of the Gazebo world to be used the world file (which is "group7.world" by default)
    worldNameArgumentLabel = 'world_name'
    declareLaunchArgumentWorldName = DeclareLaunchArgument(
        worldNameArgumentLabel,
        default_value = 'group7'
    )

    # Using "IncludeLaunchDescription" to include another launch file within this one
    # Specifically, the launch file related to starting up the Tiago simulation, that performs the following operations:
    # 1. Launches the exam Gazebo world (relying on the proper launcher tiago_exam_worlds/launch/pal_gazebo_exam.launch.py)
    # 2. Spawns the Tiago robot into the simulation (relying on the proper launcher tiago_exam/launch/tiago_spawn.launch.py)
    # 3. Starts the Tiago bringup stack (relying on the proper launcher tiago_robot/tiago_bringup/launch/tiago_bringup.launch.py)
    # 4. Optionally starts MoveIt (if the parameter "moveit" is set to "true", default is "true") (relying on the proper launcher tiago_moveit_config/launch/move_group.launch.py)
    # 5. Launches tuck_arm.py to move the arm to its initial home configuration (contained in tiago_exam/scripts)
    tiagoExamCMD = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('tiago_exam'),
                'launch',
                'tiago_exam.launch.py'
            )
        ]),
        # Setting up the correct parameters for the launch file
        launch_arguments={
            'world_name': LaunchConfiguration(worldNameArgumentLabel),
            'moveit': 'true'
        }.items()
    )

    # Again, using "IncludeLaunchDescription" to include another launch file within this one
    # Specifically, the launch file related to starting up the Nav2 & RViz stack for SLAM and mapping purposes, that performs the following operations:
    #
    # If is_public_sim is True, the launcher uses the following standard Nav2 & RViz Stack bringup (properly customized for the usage of Tiago):
    #   1. Launch of the Nav2 Stack bringup (relying on the proper launcher nav2_bringup/launch/bringup_launch.py, contained in the Nav2 package "nav2_bringup" installed within ROS2 Humble), parameterized with:
    #   - "params_file" parameter: set to the path of a pre-existing YAML configuration file "tiago_nav_public_sim.yaml" (which is "tiago_ws/src/tiago_navigation/tiago_2dnav/params/tiago_nav_public_sim.yaml")
    #   - "map" parameter: set to the path of a pre-existing map accordingly to the "world_name" launcher-parameter (which is pmb2_navigation/pmb2_maps/configurations/<world_name>/map.yaml)
    #   - "use_sim_time" parameter: set to "True", in order to make Nav2 Stack nodes using the simulation clock (instead of the real machine clock)
    #   2. Launch of RViz coherently with the Nav2 Stack (relying on the proper launcher nav2_bringup/launch/rviz_launch.py contained in the Nav2 package "nav2_bringup" installed within ROS2 Humble), parameterized with:
    #   - "rviz_config" parameter: set to the path of a pre-existing RViz configuration file "navigation.rviz" (which is "tiago_ws/src/tiago_navigation/tiago_2dnav/config/rviz/navigation.rviz")
    # If instead is_public_sim is False, the launcher does not use the standard Nav2 Stack bringup directly.
    #
    # Instead, it relies on an already existing "PAL launcher" (pal_navigation_cfg_public/pal_navigation_cfg_bringup/launch/nav_bringup.launch.py).
    # In that case, the performed operations are the following:
    #   1. Indeed, launch of the PAL Nav2 Stack bringup (relying on the proper launcher pal_navigation_cfg_public/pal_navigation_cfg_bringup/launch/nav_bringup.launch.py), parameterized with:
    #   - "map_path" parameter: set to be the same as the "map_path" launcher-parameter 
    # Indeed, it is quite relevant to know the behavior of the PAL Nav2 Stack bringup. It behaves in the following way:
    #   1. Launch of a proper Stack accordingly to the value of the "slam" launcher-parameter:
    #   - If "slam" is set to "True", the PAL Nav2 Stack bringup launches the SLAM Stack (relying on the proper launcher nav2_bringup/launch/slam_launch.py, contained in the Nav2 package "nav2_bringup" installed within ROS2 Humble), parameterized with:
    #     - "params_file" parameter: set to the path of a pre-existing YAML configuration file (which is "tiago_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_params/params/<robot_name>_nav2.yaml")
    #   - If set to "False", the PAL Nav2 Stack bringup launches the Localization Stack (relying on the proper launcher nav2_bringup/launch/localization_launch.py, contained in the Nav2 package "nav2_bringup" installed within ROS2 Humble), parameterized with:
    #     - "params_file" parameter: set to the path of a pre-existing YAML configuration file (which is "tiago_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_params/params/<robot_name>_nav2.yaml")
    #     - "map" parameter: set as <map_path>/map.yaml (where <map_path> is the value of the "map_path" launcher-parameter)
    #   2. It remaps the "cmd_vel" topic (one of the standard topics of the Nav2 Stack) as "nav_vel" (Nav2 Stack nodes will publish velocity commands on the "nav_vel" topic, instead of the standard "cmd_vel" one)
    #   3. Launch of the Nav2 Navigation Module (relying on the proper launcher nav2_bringup/launch/navigation_launch.py, contained in the Nav2 package "nav2_bringup" installed within ROS2 Humble)
    #   4. Only if the launcher-param "rviz" is "True", launch of RViz coherently with the Nav2 Stack (relying on the proper launcher nav2_bringup/launch/rviz_launch.py contained in the Nav2 package "nav2_bringup" installed within ROS2 Humble), parameterized with:
    #   - "rviz_config" parameter: set to the path of a pre-existing RViz configuration file "navigation.rviz" (which is "tiago_ws/src/tiago_navigation/tiago_2dnav/config/rviz/navigation.rviz")
    tiagoNavBringupCMD = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('tiago_2dnav'),
                'launch',
                'tiago_nav_bringup.launch.py'
            )
        ]),
        launch_arguments={
            'is_public_sim': 'False',
            'slam': 'True',
            'rviz': 'True'
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(declareLaunchArgumentWorldName)
    ld.add_action(tiagoExamCMD)
    ld.add_action(tiagoNavBringupCMD)
    return ld