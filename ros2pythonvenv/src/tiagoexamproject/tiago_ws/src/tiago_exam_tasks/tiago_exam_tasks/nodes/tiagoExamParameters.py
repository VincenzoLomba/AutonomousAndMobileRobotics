
# This file contains some parameters that are gonna be used in developing the Project.
# BE AWARE: these parameters exist only to make development easier!

from dataclasses import dataclass
from typing import Optional, Dict, Any
from geometry_msgs.msg import Pose

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
task1FSMlaunchDelay = 30.0
task2FSMlaunchDelay = 30.0
task3FSMlaunchDelay = 30.0

exploreLiteNodeName = "explore_node"
exploreLiteExploreResumeTopicLabel = "explore/resume"
exploreLiteStatusTopicLabel = "explore/status"

tiagoArmNodeName = "tiago_arm_node"
tiagoArmActionName = "tiago_arm_action"
tiagoGripperNodeName = "tiago_gripper_node"
tiagoGripperActionName = "tiago_gripper_action"
controllerServerGetParametersServiceName = "/controller_server/get_parameters"
controllerServerSetParametersServiceName = "/controller_server/set_parameters"
collisionMonitorSetParametersServiceName = "/collision_monitor/set_parameters"

amclPoseTopic = "amcl_pose"
reinitializeGlobalLocalizationServiceName = "reinitialize_global_localization"
spinActionName = "spin"
driveOnHeadingActionName = "drive_on_heading"
headCommandTopic = "/head_controller/joint_trajectory"
jointStateTopic = "/joint_states"
panHeadJointName = "head_1_joint"
tiltHeadJointName = "head_2_joint"
navigateToPoseActionName = "navigate_to_pose"
computePathToPoseActionName = "compute_path_to_pose"
yawToleranceGoalCheckerParamName = "general_goal_checker.yaw_goal_tolerance"
headTiltTolerance = 0.02

pickLocationJSONFileName = "PickLocation.json"
placeLocationJSONFileName = "PlaceLocation.json"
pickLocationHistoryJSONFileName = "PickLocationHistory.json"
placeLocationHistoryJSONFileName = "PlaceLocationHistory.json"

autonomousLocalizationMetricsTreshold = 0.02 # 0.03 # 0.02 # 0.015 # 0.02
localizationMetricsWindowSize = 4 # 5 era cinque in origine, ho abbassato per accellerare il retry della localizzazione
localizationMetricsJumpUpResetPercent = 20
localizationMetricsMinImprovementPercent = 5.0
localizationGoodMetricConsecutiveCountRequired = 2

autonomousLocalizationSpinYaw = 3.14 / 2 # 4 # positive sign for counterclockwise rotation, negative sign for clockwise rotation
autonomousLocalizationSpinTimeAllowance = 30.0
autonomousLocalizationDriveDistance = 0.4 # 4.4 # 0.4#4.4 # 0.4#4.4#4.0#0.4
autonomousLocalizationDriveSpeed = 0.4
autonomousLocalizationDriveTimeAllowance = 6.0

headTiltDuringDiscovery = -0.12
arucoSampleDistanceThresholdDuringDiscovery = 3

fatherReferenceFrame = "map"
tiagoBaseLinkReferenceFrame = "base_link"

torsoCommandTopic = "/torso_controller/joint_trajectory"
torsoLiftJointName = "torso_lift_joint"

headTiltDuringPickAndPlace = -0.9 # 76 # -0.84 (da usare con velocità trasaz piu rapida) # -0.76 # -0.66
headPanTolerance = headTiltTolerance
headPanDuringPickAndPlace = 0.0
headPanSweepEnabled = True
headPanSweepPositions = [0.0, -0.25, 0.0, +0.25]
headPanSweepPeriod = 2 # integer
pickingWaitingTime = 5.0

platformApproachDistance = 0.2 # 0.30 # 2 # 0.25 # 0.20 (da usare con velocità trasaz piu rapida) # 0.25
platformApproachReorientation = False # True
placingApproachForwardOffset = 0.07
cubeApproachHeight = 0.50 # 0.48 #0.40 # 0.30
verticalTranslationGrasping = 0.30 # 20
safetyPlacingDeltaTranslation = 0.02

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

PICKING_JOINT_POSITIONS = [
    0.320, # 0.300,  # torso_lift_joint    0.323 m
    2.443461,        # arm_1_joint         140°
    -0.785398,       # arm_2_joint         -45°
    0.837758,        # arm_3_joint         48°
    2.164208,        # arm_4_joint         124°
    1.047198,        # arm_5_joint         0°
    1.308997,        # arm_6_joint         0°
    0.0,             # arm_7_joint         12°
]

TRANSPORTATION_JOINT_POSITIONS = PICKING_JOINT_POSITIONS # Transport configuration defined as the same to the picking one, BUT it can be freely changed

@dataclass(frozen = True)
class MarkerInfo:
    # This class simply represents the relevant information about an Aruco Marker
    markerSize: float # Physical marker side length in meters.
    markerID: int # Numerical ArUco marker ID.
    markerNickname: str # Human-readable semantic name, e.g. "pick" or "place", or "cube63" or "cube582".
    markerFrame: str # TF frame associated with this marker.
    markerReferenceFrame: str # Reference frame used to express the marker pose, e.g. "map".
    gazeboModelName: Optional[str] = None # Name of the Gazebo model of the object associated with this marker, IF any.
    gazeboModelLinkName: Optional[str] = None # Name of the Gazebo model LINK of the object associated with this marker, IF any.
    def getNamespace(self) -> str: return "aruco_" + self.markerNickname
    def getNodeName(self) -> str: return self.getNamespace() + "_single_node"
    def getTopicTF(self) -> str: return "/" + self.getNamespace() + "/" + self.getNodeName() + "/transform"

# Note that the cubes are expressed to be Gazebo Objects in the form:
# model: aruco_cube_exam_id{ID}
# └── link: link
#     ├── collision geometry
#     ├── visual geometry
#     └── inertial/mass

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
cube63Marker = MarkerInfo(
    markerSize = 0.07,
    markerID = 63,
    markerNickname = "cube63",
    markerFrame = "aruco_marker_frame_63",
    markerReferenceFrame = "map",
    gazeboModelName = "aruco_cube_exam_id63",
    gazeboModelLinkName = "link"
)
cube582Marker = MarkerInfo(
    markerSize = 0.07,
    markerID = 582,
    markerNickname = "cube582",
    markerFrame = "aruco_marker_frame_582",
    markerReferenceFrame = "map",
    gazeboModelName = "aruco_cube_exam_id582",
    gazeboModelLinkName = "link"
)
cubes = [cube63Marker, cube582Marker]

@dataclass
class ArucoPoseEstimate:
    # This class represents the final estimated pose of an ArUco-marked location after aggregating all collected samples
    found: bool # Whether at least one valid marker sample was collected.
    marker_id: int # Numerical ID of the ArUco marker associated with this location.
    label: str # Semantic role of the location, e.g. "pick" or "place".
    frame_id: str # Reference frame in which the estimated pose is expressed, typically "map" ("map" is indeed used as a default value).
    marker_frame: str # TF frame name associated with the specific ArUco marker.
    sample_count: int # Number of samples used to compute the estimate.
    pose: Optional[Pose] # Estimated marker pose as a geometry_msgs/Pose message, if found.
    # position: Optional[Dict[str, float]] = None # Estimated marker position {"x": ..., "y": ..., "z": ...}, if found.
    # orientation: Optional[Dict[str, float]] = None # Estimated marker orientation {"x": ..., "y": ..., "z": ..., "w": ...}, if found.
    def toDict(self) -> Dict[str, Any]:
        return {
            "found": bool(self.found),
            "marker_id": int(self.marker_id),
            "label": str(self.label),
            "frame_id": str(self.frame_id),
            "marker_frame": str(self.marker_frame),
            "sample_count": int(self.sample_count),
            "pose": None if self.pose is None else {
                "position": {
                    "x": float(self.pose.position.x),
                    "y": float(self.pose.position.y),
                    "z": float(self.pose.position.z),
                },
                "orientation": {
                    "x": float(self.pose.orientation.x),
                    "y": float(self.pose.orientation.y),
                    "z": float(self.pose.orientation.z),
                    "w": float(self.pose.orientation.w),
                },
            },
        }
