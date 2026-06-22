
# This file contains some parameters that are gonna be used in developing the Project.
# BE AWARE: these parameters exist only to make development easier!

from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, List

gazeboWorldName = "group7"

fsmTimerPeriodParameterName = "fsm_timer_period"
fsmTimerPeriodParameterDefaultValue = 0.5
mapSavePathParameterName = "map_path"
mapSavePathParameterDefaultValue = "/home/vincenzo/Documenti/AMR/ros2pythonvenv/src/tiagoexamproject/map" # "/tmp/tiagomap/"
mapSaveNameParameterName = "map_name"
mapSaveNameParameterDefaultValue = "map"

nav2SaveMapServiceName = "/map_saver/save_map"
nav2MapTopic = "map"

tasksFSMFolderName = "tiago_exam_tasks"
task1FSMNodeName = "task1_fsm_node"
task2FSMNodeName = "task2_fsm_node"
task3FSMNodeName = "task3_fsm_node"
task1FSMlaunchDelay = 15.0
task2FSMlaunchDelay = 15.0
task3FSMlaunchDelay = 15.0

exploreLiteNodeName = "explore_node"
exploreLiteExploreResumeTopicLabel = "explore/resume"
exploreLiteStatusTopicLabel = "explore/status"

tiagoArmNodeName = "tiago_arm_node"
tiagoArmActionName = "tiago_arm_action"
controllerServerGetParametersServiceName = "/controller_server/get_parameters"
controllerServerSetParametersServiceName = "/controller_server/set_parameters"

amclPoseTopic = "amcl_pose"
reinitializeGlobalLocalizationServiceName = "reinitialize_global_localization"
spinActionName = "spin"
driveOnHeadingActionName = "drive_on_heading"
headCommandTopic = "/head_controller/joint_trajectory"
jointStateTopic = "/joint_states"
panHeadJointName = "head_1_joint"
tiltHeadJointName = "head_2_joint"
yawToleranceGoalCheckerParamName = "general_goal_checker.yaw_goal_tolerance"

autonomousLocalizationMetricsTreshold = 0.02 # 0.015 # 0.02
localizationMetricsWindowSize = 5
localizationMetricsJumpUpResetPercent = 20
localizationMetricsMinImprovementPercent = 5.0
localizationGoodMetricConsecutiveCountRequired = 2

autonomousLocalizationSpinYaw = 3.14 / 2 # 4 # positive sign for counterclockwise rotation, negative sign for clockwise rotation
autonomousLocalizationSpinTimeAllowance = 30.0
autonomousLocalizationDriveDistance = 0.4 # 4.4 # 0.4#4.4 # 0.4#4.4#4.0#0.4
autonomousLocalizationDriveSpeed = 0.4
autonomousLocalizationDriveTimeAllowance = 6.0

headTiltDuringDiscovery = -0.12
fatherReferenceFrame = "map"

HOME_JOINT_POSITIONS = [
    0.0,       # torso_lift_joint   0.0m
    2.478368,  # arm_1_joint        142°
    -1.343904, # arm_2_joint        -77°
    0.506145,  # arm_3_joint        29°
    1.972222,  # arm_4_joint        113°
    -1.745329, # arm_5_joint        -100°
    1.396263,  # arm_6_joint        80°
    0.226893,  # arm_7_joint        13°
]

@dataclass(frozen = True)
class MarkerInfo:
    markerSize: float # Physical marker side length in meters.
    markerID: int # Numerical ArUco marker ID.
    markerNickname: str # Human-readable semantic name, e.g. "pick" or "place".
    markerFrame: str # TF frame associated with this marker.
    markerReferenceFrame: str # Reference frame used to express the marker pose, e.g. "map".

pickLocationMarker = MarkerInfo(
    markerSize = 0.25,
    markerID = 26,
    markerNickname = "pick",
    markerFrame = "aruco_marker_frame_26",
    markerReferenceFrame = "map"
)
placeLocationMarker = MarkerInfo(
    markerSize = 0.25,
    markerID = 238,
    markerNickname = "place",
    markerFrame = "aruco_marker_frame_238",
    markerReferenceFrame = "map"
)

@dataclass
class ArucoSample:
    # This simple class represents the single sample collected via an Aruco Marker detection
    stamp_sec: int # Seconds part of the ROS timestamp associated with the transform message.
    stamp_nanosec: int # Nanoseconds part of the ROS timestamp associated with the transform message.
    frame_id: str # Reference frame in which the marker pose is expressed, typically "map".
    child_frame_id: str # TF child frame associated with the detected ArUco marker.
    position: List[float] # Marker position [x, y, z] expressed in frame_id, in meters.
    quaternion: List[float] # Normalized marker orientation [x, y, z, w] expressed in frame_id.
    robot_position: Optional[Dict[str, float]] # Estimated robot position {"x": ..., "y": ...} from AMCL at detection time, if available.
    robot_marker_distance_2d: Optional[float] # Planar distance between the robot and the marker in the map frame, if AMCL is available.
    robot_marker_bearing_map: Optional[float] # Planar bearing angle from the robot to the marker in the map frame, in radians, if AMCL is available.

@dataclass
class ArucoLocationEstimate:
    # This class represents the final estimated pose of an ArUco-marked location after aggregating all collected samples
    found: bool # Whether at least one valid marker sample was collected.
    marker_id: int # Numerical ID of the ArUco marker associated with this location.
    label: str # Semantic role of the location, e.g. "pick" or "place".
    frame_id: str # Reference frame in which the estimated pose is expressed, typically "map" ("map" is indeed used as a default value).
    marker_frame: str # TF frame name associated with the specific ArUco marker.
    sample_count: int # Number of samples used to compute the estimate.
    position: Optional[Dict[str, float]] = None # Estimated marker position {"x": ..., "y": ..., "z": ...}, if found.
    orientation: Optional[Dict[str, float]] = None # Estimated marker orientation {"x": ..., "y": ..., "z": ..., "w": ...}, if found.
    def to_dict(self) -> Dict[str, Any]: return asdict(self)