
# This is the launch file for Project Task3: sequential cubes transportation from pick to place locations

# Necessary imports
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from tiago_exam_tasks.nodes import tiagoExamParameters
from launch.actions import TimerAction

def generate_launch_description():

    # Remember: use "DeclareLaunchArgument" to declare a launch argument/parameter (that can be set from the command line when launching the file)

    # This is the name of the Gazebo world to be used the world file (which is gonna be "group7.world" by default)
    worldNameArgumentLabel = 'world_name'
    declareLaunchArgumentWorldName = DeclareLaunchArgument(
        worldNameArgumentLabel,
        default_value = tiagoExamParameters.gazeboWorldName
    )
    # This is the maximum time the Tiago spawner waits before successfully spawning the robot.
    tiagoSpawnTimeoutArgumentLabel = 'tiagoSpawnTimeout'
    declareLaunchArgumentTiagoSpawnTimeout = DeclareLaunchArgument(
        tiagoSpawnTimeoutArgumentLabel,
        default_value = '60.0'
    )
    # This is the X coordinate of the Tiago robot spawn point in the Gazebo world (default is 0.0).
    tiagoSpawnCoordinateXArgumentLabel = 'tiagoSpawnCoordinateX'
    declareLaunchArgumentTiagoSpawnCoordinateX = DeclareLaunchArgument(
        tiagoSpawnCoordinateXArgumentLabel,
        default_value = '0.0'
    )
    # This is the Y coordinate of the Tiago robot spawn point in the Gazebo world (default is -1.3).
    tiagoSpawnCoordinateYArgumentLabel = 'tiagoSpawnCoordinateY'
    declareLaunchArgumentTiagoSpawnCoordinateY = DeclareLaunchArgument(
        tiagoSpawnCoordinateYArgumentLabel,
        default_value = '-1.3'
    )
    # This is the Z coordinate of the Tiago robot spawn point in the Gazebo world (default is 0.0).
    tiagoSpawnCoordinateZArgumentLabel = 'tiagoSpawnCoordinateZ'
    declareLaunchArgumentTiagoSpawnCoordinateZ = DeclareLaunchArgument(
        tiagoSpawnCoordinateZArgumentLabel,
        default_value = '0.0'
    )
    # This is the period of the Task3 FSM timer
    declareLaunchArgumentFSMtimerPeriod = DeclareLaunchArgument(
        tiagoExamParameters.fsmTimerPeriodParameterName,
        default_value = str(tiagoExamParameters.fsmTimerPeriodParameterDefaultValue)
    )
    # This is the path (name excluded) where the map to be used (for running the discovery policy) is gonna be stored
    declareLaunchArgumentSavedMapPath = DeclareLaunchArgument(
        tiagoExamParameters.mapSavePathParameterName,
        default_value = tiagoExamParameters.mapSavePathParameterDefaultValue
    )
    # This is the name of the map file (without the path) that is gonna be used
    declareLaunchArgumentSavedMapName = DeclareLaunchArgument(
        tiagoExamParameters.mapSaveNameParameterName,
        default_value = tiagoExamParameters.mapSaveNameParameterDefaultValue
    )

    # Remember: use "IncludeLaunchDescription" to include another launch file within this one

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
            'moveit': 'true',
            'tiagoSpawnTimeout': LaunchConfiguration(tiagoSpawnTimeoutArgumentLabel),
            'tiagoSpawnCoordinateX': LaunchConfiguration(tiagoSpawnCoordinateXArgumentLabel),
            'tiagoSpawnCoordinateY': LaunchConfiguration(tiagoSpawnCoordinateYArgumentLabel),
            'tiagoSpawnCoordinateZ': LaunchConfiguration(tiagoSpawnCoordinateZArgumentLabel)
        }.items()
    )

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
            'slam': 'False',
            'rviz': 'True',
            'map_path': LaunchConfiguration(tiagoExamParameters.mapSavePathParameterName),
            'cmd_vel_smoothed_remap_topic': 'cmd_vel_unsafe' # This is UNused in Task3
        }.items()
    )

    # Using "Node" to include within this launch file the Node that exposes the Action Server to control and move the Tiago's arm
    tiagoArmNodeCMD = Node(
        package = tiagoExamParameters.tasksFSMFolderName,
        executable = tiagoExamParameters.tiagoArmNodeName,
        name = tiagoExamParameters.tiagoArmNodeName,
        output = 'screen', # This makes the node's log messages appear in the terminal, which can be useful for debugging and monitoring
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # Using "Node" to include within this launch file the Node that exposes the Action Server to control and move the Tiago's gripper
    tiagoGripperNodeCMD = Node(
        package = tiagoExamParameters.tasksFSMFolderName,
        executable = tiagoExamParameters.tiagoGripperNodeName,
        name = tiagoExamParameters.tiagoGripperNodeName,
        output = 'screen', # This makes the node's log messages appear in the terminal, which can be useful for debugging and monitoring
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # Using "Node" to include within this launch file the Node that implements the Collision Monitor.
    # When active, ti behaves as a filter for the original cmd_vel topic (exploiting the odometry), as anticipated in the comments above.
    # Also, this is a "LifecycleNode", that means (from Ros2 Humble documentation) that is not sufficient to run it: it also has to be put in an "active" state explicitly.
    # Note that the behaviour of that Collision Monitor Node is described in the file tiago_exam_tasks/config/collision_monitor.yaml
    collisionMonitorCMD = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        remappings=[
            ('cmd_vel_safe', 'cmd_vel')
        ],
        parameters=[
            {'use_sim_time': True},
            os.path.join(
                get_package_share_directory('tiago_exam_tasks'),
                'config',
                'collision_monitor.yaml'
            )
        ]
    )

    # Using "Node" to include within this launch file the Node that that (as just commented above) explicitly puts on an "active" state the Collision Monitor Node.
    collisionMonitorLifecycleManagerCMD = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['collision_monitor'],
            'bond_timeout': 0.0
        }]
    )

    # Using "Node" to include within this launch file the Node that implements the Aruco Marker detection for the pick location (marker ID 63)
    cube63ArucoNodeCMD = Node(
        package='aruco_ros',
        executable="single",
        namespace=tiagoExamParameters.cube63Marker.getNamespace(),
        name=tiagoExamParameters.cube63Marker.getNodeName(),
        remappings=[
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
        ],
        parameters=[{
            "use_sim_time": True,
            "image_is_rectified": True,
            "marker_size": tiagoExamParameters.cube63Marker.markerSize,
            "marker_id": tiagoExamParameters.cube63Marker.markerID,
            "reference_frame": tiagoExamParameters.cube63Marker.markerReferenceFrame,
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": tiagoExamParameters.cube63Marker.markerFrame,
            "corner_refinement": "LINES",
        }]
    )

    # Using "Node" to include within this launch file the Node that implements the Aruco Marker detection for the pick location (marker ID 582)
    cube582ArucoNodeCMD = Node(
        package='aruco_ros',
        executable="single",
        namespace=tiagoExamParameters.cube582Marker.getNamespace(),
        name=tiagoExamParameters.cube582Marker.getNodeName(),
        remappings=[
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
        ],
        parameters=[{
            "use_sim_time": True,
            "image_is_rectified": True,
            "marker_size": tiagoExamParameters.cube582Marker.markerSize,
            "marker_id": tiagoExamParameters.cube582Marker.markerID,
            "reference_frame": tiagoExamParameters.cube582Marker.markerReferenceFrame,
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": tiagoExamParameters.cube582Marker.markerFrame,
            "corner_refinement": "LINES",
        }]
    )

    # Using "Node" to include within this launch file the Node that implements the FSM that implements the logic that executes the Task3
    # Wrapping it within a "TimerAction" to delay its start of some pre-choosen seconds
    task3FSMNodeCMDdelayed = TimerAction(
        period = tiagoExamParameters.task3FSMlaunchDelay,
        actions= [
            Node(
                package = 'tiago_exam_tasks',
                executable = 'task3_fsm_node',
                name = 'task3_fsm_node',
                output = 'screen', # This makes the node's log messages appear in the terminal, which can be useful for debugging and monitoring
                parameters = [
                    {'use_sim_time': True},
                    {tiagoExamParameters.fsmTimerPeriodParameterName: LaunchConfiguration(tiagoExamParameters.fsmTimerPeriodParameterName)},
                    {tiagoExamParameters.mapSavePathParameterName: LaunchConfiguration(tiagoExamParameters.mapSavePathParameterName)},
                    {tiagoExamParameters.mapSaveNameParameterName: LaunchConfiguration(tiagoExamParameters.mapSaveNameParameterName)}
                ]
            )
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declareLaunchArgumentWorldName)
    ld.add_action(declareLaunchArgumentTiagoSpawnTimeout)
    ld.add_action(declareLaunchArgumentTiagoSpawnCoordinateX)
    ld.add_action(declareLaunchArgumentTiagoSpawnCoordinateY)
    ld.add_action(declareLaunchArgumentTiagoSpawnCoordinateZ)
    ld.add_action(declareLaunchArgumentFSMtimerPeriod)
    ld.add_action(declareLaunchArgumentSavedMapPath)
    ld.add_action(declareLaunchArgumentSavedMapName)
    ld.add_action(tiagoExamCMD)
    ld.add_action(tiagoNavBringupCMD)
    ld.add_action(tiagoArmNodeCMD)
    ld.add_action(tiagoGripperNodeCMD)
    ld.add_action(collisionMonitorCMD)
    ld.add_action(collisionMonitorLifecycleManagerCMD)
    ld.add_action(cube63ArucoNodeCMD)
    ld.add_action(cube582ArucoNodeCMD)
    ld.add_action(task3FSMNodeCMDdelayed)
    return ld