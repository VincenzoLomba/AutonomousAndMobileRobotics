
# Custom-created launcher file for a custom ROS2 Package!

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL'] # getting the TurtleBOT3 model Linux environment variable value!

def generate_launch_description():
  
  # LaunchConfiguration allows you to define parameters that can be set from the command line when launching the file!
  # Let's indeed define some parameters and their default values
  use_sim_time = LaunchConfiguration('use_sim_time', default='True')
  x_pose = LaunchConfiguration('x_pose', default='-2.0')
  y_pose = LaunchConfiguration('y_pose', default='-0.5')

  # We need to include the parameters file of the turtlebot3_navigation2 package (the already existing one)
  param_file_name = TURTLEBOT3_MODEL + '.yaml'
  param_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'humble', param_file_name)

  # We need to include the default nav2 configuration file too
  rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

  # Using class "IncludeLaunchDescription" to include ANOTHER launch file within THIS one (of course we need to know its location and name)!
  # Indeed, until now the current launch file simply embeds within itself the other launch file related to the TurtleBOT3 world!
  turtlebot3_world_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
      '/turtlebot3_world.launch.py'
    ]),
    launch_arguments={
      'use_sim_time': use_sim_time,
      'x_pose': x_pose,
      'y_pose': y_pose
    }.items()
  )

  # Again, using class "IncludeLaunchDescription" to include ANOTHER launch file.
  # This time we include the nav2_bringup launch file, which will launch all the nodes needed for navigation and SLAM!
  navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
      '/bringup_launch.py'
    ]),
    launch_arguments={
      'map' : '/home/vincenzo/Documenti/AMR/ros2pythonvenv/src/lab3workspace/data/map.yaml',
      'use_sim_time': use_sim_time,
      'slam' : 'True',
      'params_file': param_dir
    }.items(),
  )

  # Then, we also need RViz Node to visualize the map and the robot's pose!
  # Notice that in that case we are not anymore referring to a whole package and its launch file, but directly to a specific Node (the RViz2 one)!
  # Of course, to refear to a specific Node we need to know its package and executable names!
  rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_dir],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
  )

  # Finally, we include the explore_lite package launch file to enable autonomous exploration!
  # Again, we use the IncludeLaunchDescription class to include the explore.launch.py file within this one!
  # Of course, we still need to know the package name and the launch file name.
  explore_lite_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('explore_lite'), 'launch'),
      '/explore.launch.py'
    ]),
    launch_arguments={
      'use_sim_time': use_sim_time
    }.items(),
  )

  # At the end, we create a LaunchDescription object and populate it with the various commands we created!
  ld = LaunchDescription()
  ld.add_action(turtlebot3_world_cmd)
  ld.add_action(navigation_cmd)
  ld.add_action(rviz_cmd)
  ld.add_action(explore_lite_cmd)
  return ld