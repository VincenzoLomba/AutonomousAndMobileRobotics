import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    return_to_init = LaunchConfiguration("return_to_init")
    start_exploration_immediately = LaunchConfiguration(
        "start_exploration_immediately")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )
    # Note: return_to_init moved from params.yaml to launch argument.
    # Default true: robot returns to starting pose after exploration completes.
    declare_return_to_init_argument = DeclareLaunchArgument(
        "return_to_init",
        default_value="true",
        description=(
            "If true, the robot navigates back to its initial pose after exploration "
            "completes. If false, the robot stops at its last position."
        ),
    )
    # Note: controls whether exploration starts immediately on node launch.
    # Default true: original ExploreLite behaviour (start immediately).
    # Set to false to start paused and wait for True on /explore/resume.
    declare_start_immediately_argument = DeclareLaunchArgument(
        "start_exploration_immediately",
        default_value="true",
        description=(
            "If true, exploration begins immediately when the node starts (original "
            "behaviour). If false, the node starts paused and waits for a True message "
            "on the /explore/resume topic."
        ),
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="explore_lite",
        name="explore_node",
        namespace=namespace,
        executable="explore",
        parameters=[
            config,
            {
                "use_sim_time": use_sim_time,
                "return_to_init": return_to_init,                            # MOD 18
                "start_exploration_immediately": start_exploration_immediately,  # MOD 17
            },
        ],
        output="screen",
        remappings=remappings,
    )
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_return_to_init_argument)
    ld.add_action(declare_start_immediately_argument)
    ld.add_action(node)
    return ld
