
# This Python files defines a ROS2 Humble Node which implements the FSM that implements the logic that executes the Task2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tiago_exam_interfaces.action import TiagoArm
from enum import Enum
from . import tiagoExamParameters as params
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty
from nav2_msgs.action import Spin
from builtin_interfaces.msg import Duration
from nav2_msgs.action import DriveOnHeading
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.parameter import Parameter
import random
import os
import json
import math
from pathlib import Path
import numpy as np
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from scipy.spatial.transform import Rotation as SciPyRotation
from . import discoveryPlanner
from dataclasses import asdict
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, asdict
from geometry_msgs.msg import Pose

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

class Task2FSMState(Enum):
    # Autonomous localization
    TUCK_ARM = 1
    WAIT_ARM_TUCKED = 2
    TRIGGER_LOCALIZATION_RESTART = 3
    WAIT_LOCALIZATION_RESTARTED = 4
    EVALUATING_LOCALIZATION = 5
    LOCALIZING = 6
    # Pick and place locations discovery
    SET_HEAD_FOR_DISCOVERY = 7
    WAIT_HEAD_FOR_DISCOVERY = 8
    DISCOVERING  = 9
    DISCOVERY_SET_YAW_TOLERANCE = 18
    DISCOVERY_WAIT_YAW_TOLERANCE_SET = 19
    DISCOVERY_COMPUTE_PATH = 12
    DISCOVERY_WAIT_PATH = 13
    DISCOVERY_SPIN_TO_PATH_HEADING = 14
    DISCOVERY_WAIT_SPIN = 15
    DISCOVERY_NAVIGATE_TO_GOAL = 16
    DISCOVERY_WAIT_NAVIGATION = 17
    DISCOVERY_RESTORE_YAW_TOLERANCE = 20
    DISCOVERY_WAIT_YAW_TOLERANCE_RESTORED = 21
    DISCOVERY_FINISHED = 22

class Task2FSMNode(Node):

    def __init__(self):

        super().__init__(params.task2FSMNodeName)
        self.get_logger().info(f"Starting {params.task2FSMNodeName} initialization...")

        self.lastAMCLMsg = None # Variable storing the last message received from the "amcl_pose" topic
        self.latestJointState = None # Variable storing the last message received from the "joint_states" topic

        self.collectArucoSamples = False # A variable indicating whether the Node is actually collectiong the Aruco Marker samples
        self.pickArucoSamples = [] # List of the collected Aruco Marker samples for the pick location (marker ID 26)
        self.placeArucoSamples = [] # List of the collected Aruco Marker samples for the place location (marker ID 238)
        self.arucoLocationsSaved = False # A simple FLAG variable used to notify the successful saving of the pick/place Aruco Marker location data (after the discovery)

        self.armTuckGoalDone = False
        self.armTuckGoalSucceeded = False
        self.globalLocalizationResetDone = False
        self.globalLocalizationResetSucceeded = False

        # A list in which the hystory of the periodically computed localization metric is progressively stored
        self.localizationMetricsHistory = []
        # The list of the localization metric hystory is limited to the value of this variable
        self.localizationMetricsWindowSize = params.localizationMetricsWindowSize
        # If the localization metric jumps up more than this percentage, then the history of the localization metric is reset (it's assumed that the localization procedure has just performed a big correction and its evaluation should therefore be reset)
        self.localizationMetricsJumpUpResetPercent = params.localizationMetricsJumpUpResetPercent
        # The minimum required improvement percentage of the localization metric (at each new computation of it)
        self.localizationMetricsMinImprovementPercent = params.localizationMetricsMinImprovementPercent

        self.collisionMonitorRequestSent = False # A FLAG variable used to notify that a request to the collision monitor has been sent (to enable OR disable it)
        self.collisionMonitorResponseReceived = False # A FLAG variable used to notify that a response from the collision monitor has been received (after a request to enable OR disable it); note that the response may be both a success or a failure (this is sort of a "done" flag) 
        self.collisionMonitorEnabled = False # A simple FLAG that indicated if the Collision Monitor is currently enabled or disabled

        self.localizationPhase = None # During the localization phase, a Spin and a DriveOnHeading Actions are periodically alternated; this variable indicates the next Action to be performed: 0 for Spin, 1 for DriveOnHeading.
        self.localizationGoodMetricConsecutiveCountRequired = params.localizationGoodMetricConsecutiveCountRequired # Not only the threshold of the localization metric has to be satisfied, but also it has to be satisfied for a certain number of consecutive times (this variable defines that number)
        self.localizationGoodMetricConsecutiveCount = 0 # This variable is a simple counter that counts how many times the localization metric has been consecutively good
        self.robotXY = (0.0, 0.0) # The location of the robot (that will be correctly updated after the conclusion of the autonomous localization procedure)
        
        self.localizationSpinDone = False
        self.localizationSpinSucceeded = False
        self.localizationDriveOnHeadingDone = False
        self.localizationDriveOnHeadingSucceeded = False

        self.RANDOMWALKlocalization = False  # In a previous version of the Task2 code, the autonomous localization was performed via a random-walk-like policy.
        # Anyway, it has been experimentally verified as more effective a different policy: in-place rotation with eventual DriveOnHeading correction.
        # As it is explained more in detail in the following parts of the code, in the final version this second policy has been adopted.

        # When modifying the head tilt position, this tolerance (in radians) is used to wait for the new positino to be actually reached
        self.headTiltTolerance = params.headTiltTolerance

        self.discoveryOrderedKeypoints = [] # This list will contain the ordered list of keypoints used for the discovery phase (in terms of x,y coordinates)
        self.discoveryOrderedGoals = [] # Same as the previous list, but in terms of NavigateToPose.Goal objects to be used with the NavigateToPose Action Server
        self.discoveryCurrentGoalIndex = 0 # The index of the keypoint that is still current being ddiscovered
        
        self.discoveryYawParamDone = False
        self.discoveryYawParamSucceeded = False
        self.yawGoalToleranceParamName = params.yawToleranceGoalCheckerParamName # This is the name of the parameter related to the final YAW Goal Checking
        self.originalYawGoalTolerance = None # During the discovery procedure, the YAW tolerance of the Nav2 Goal Checker is altered to be practically disabled.
        # That said, the original value is stored in this variable "originalYawGoalTolerance" to be later on restored (at the end of the discovery phase itself).

        self.discoveryPathDone = False
        self.discoveryPathSucceeded = False
        self.discoveryComputedPath = None # During the discovery process, this variable stores the last computed global path (towards the current keypoint)
        
        self.discoverySpinDone = False
        self.discoverySpinSucceeded = False
        self.discoveryNavDone = False
        self.discoveryNavSucceeded = False
        self.discoveryRestoreYawDone = False
        self.discoveryRestoreYawSucceeded = False
        
        self.shouldShutdown = False

        # Initialize the Action Client for the TiagoArm Action Server (exposed by the TiagoArm Node) to be able to send goal for the arm motion
        self.tiagoArmActionClient = ActionClient(self, TiagoArm, params.tiagoArmActionName) # Action Client
        # Subscribe to the "amcl_pose" topic to get the robot's estimated position (AMCL = Adaptive Monte Carlo Localization) (note that this is a PLANAR localizator)
        self.AMCLPoseSubscription = self.create_subscription(PoseWithCovarianceStamped, params.amclPoseTopic, self.amclCallback, 10)
        # Subscribe to the "joint_states" topic to get the current joint states of the robot
        self.jointStateSubscription = self.create_subscription(JointState, params.jointStateTopic, self.jointStateCallback, 10)
        # Subscribe to the Aruco Marker detection topics for both pick and place locations to get the detected marker poses
        self.pickArucoSubscription = self.create_subscription(TransformStamped, params.pickLocationMarker.getTopicTF(), self.pickArucoCallback, 10)
        self.placeArucoSubscription = self.create_subscription(TransformStamped, params.placeLocationMarker.getTopicTF(), self.placeArucoCallback, 10)
        # Initialize the Publisher for commanding the head position (its tilt position) AKA publisher to the "/head_controller/joint_trajectory" topic
        self.headCommandPublisher = self.create_publisher(JointTrajectory, params.headCommandTopic, 10)
        # Initialize the Service Client "reinitialize_global_localization" to be used to reinitialize in a clean way the global autonomous localization process (that will be later on managed via AMCL)
        self.reinitializeGlobalLocalizationClient = self.create_client(Empty, params.reinitializeGlobalLocalizationServiceName)
        # Initialize the Action Client for the Spin Action Server provided by the Nav2 Stack
        self.spinActionClient = ActionClient(self, Spin, params.spinActionName)
        # Initialize the Action Client for the DriveOnHeading Action Server provided by the Nav2 Stack
        self.driveOnHeadingActionClient = ActionClient(self, DriveOnHeading, params.driveOnHeadingActionName)
        # Initialize the Service Client for enabling/disabling the collision monitor
        self.collisionMonitorEnabler = self.create_client(SetParameters, params.collisionMonitorSetParametersServiceName)
        # Initialize the Service Clients for getting and setting parameters on the controller server (later on used to online get and set the yaw_goal_tolerance value of the Goal Checker used within the Nav2 Stack)
        self.controllerServerGetParametersClient = self.create_client(GetParameters, params.controllerServerGetParametersServiceName)
        self.controllerServerSetParametersClient = self.create_client(SetParameters, params.controllerServerSetParametersServiceName)
        # Initialize the Action Client for the ComputePathToPose Action Server provided by the Nav2 Stack
        self.computePathActionClient = ActionClient(self, ComputePathToPose, params.computePathToPoseActionName)
        # Initialize the Action Client for the NavigateToPose Action Server provided by the Nav2 Stack
        self.navigateToPoseActionClient = ActionClient(self, NavigateToPose, params.navigateToPoseActionName)

        # Initialize the internal state
        self.currentFSMstate = Task2FSMState.TUCK_ARM

        # Retrieve the map path parameter
        mapSavePathDefaultValue = params.mapSavePathParameterDefaultValue
        self.declare_parameter(params.mapSavePathParameterName, Parameter.Type.STRING)
        self.savedMapPath = self.get_parameter(params.mapSavePathParameterName).value
        if self.savedMapPath is None or str(self.savedMapPath).strip() == "":
            self.get_logger().warn(f"None, empty or invalid {params.mapSavePathParameterName} parameter, falling back to default value '{mapSavePathDefaultValue}'.")
            self.savedMapPath = mapSavePathDefaultValue
        else: self.savedMapPath = str(self.savedMapPath).strip()

        # Retrieve the map name parameter
        mapSaveNameDefaultValue = params.mapSaveNameParameterDefaultValue
        self.declare_parameter(params.mapSaveNameParameterName, Parameter.Type.STRING)
        self.savedMapName = self.get_parameter(params.mapSaveNameParameterName).value
        if self.savedMapName is None or str(self.savedMapName).strip() == "":
            self.get_logger().warn(f"None or empty {params.mapSaveNameParameterName} parameter, falling back to default value '{mapSaveNameDefaultValue}'.")
            self.savedMapName = mapSaveNameDefaultValue
        else: self.savedMapName = str(self.savedMapName).strip()

        # Retrieve the FSM timer period parameter
        fsmTimerPeriodDefaultValue = params.fsmTimerPeriodParameterDefaultValue
        self.declare_parameter(params.fsmTimerPeriodParameterName, Parameter.Type.DOUBLE)
        fsmTimerPeriod = self.get_parameter(params.fsmTimerPeriodParameterName).value
        if fsmTimerPeriod is None or float(fsmTimerPeriod) <= 0.0:
            self.get_logger().warn(f"Invalid FSM timer period ({fsmTimerPeriod}), falling back to default value {fsmTimerPeriodDefaultValue} secs.")
            fsmTimerPeriod = fsmTimerPeriodDefaultValue
        else: fsmTimerPeriod = float(fsmTimerPeriod)

        self.FSMtimer = self.create_timer(fsmTimerPeriod, self.stepUpFSM) # A timer created like that (alias with self.create_timer) is gonna be spun as soon as this node is spun via rclpy.spin(); also remeber that a ROS2 timer always waits for the end of the previous callback execution before triggering the next one
        self.get_logger().info(f"Task2 FSM node initialized. Initial state: {self.currentFSMstate.name}.")
    
    def logInfoWithStatus(self, message): self.get_logger().info(f"[{self.currentFSMstate.name}] {message}")
    def logWarnWithStatus(self, message): self.get_logger().warn(f"[{self.currentFSMstate.name}] {message}")
    def logErrorWithStatus(self, message): self.get_logger().error(f"[{self.currentFSMstate.name}] {message}")

    def amclCallback(self, msg): self.lastAMCLMsg = msg
    def jointStateCallback(self, msg): self.latestJointState = msg
    def pickArucoCallback(self, msg: TransformStamped): self.storeArucoSample(msg, self.pickArucoSamples, 'pick', 26)
    def placeArucoCallback(self, msg: TransformStamped): self.storeArucoSample(msg, self.placeArucoSamples, 'place', 238)

    def storeArucoSample(self, msg: TransformStamped, samples: list, label: str, marker_id: int):
        # This method takes care of the collected Aruco Marker samples (for both pick and place locations) and stores them in the corresponding list
        if not self.collectArucoSamples: return
        t = msg.transform.translation
        q = msg.transform.rotation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm <= 1e-9:
            self.logWarnWithStatus(f"Ignoring {label} marker {marker_id} sample with invalid [almost] zero-norm quaternion.")
            return
        robotPosition = None
        robotMarkerDistance2D = None
        robotMarkerBearingMap = None
        if self.lastAMCLMsg is not None:
            # Retrieving and exploiting the current/last AMCL pose of the robot in the map
            robotX = float(self.lastAMCLMsg.pose.pose.position.x)
            robotY = float(self.lastAMCLMsg.pose.pose.position.y)
            robotPosition = {'x': robotX, 'y': robotY}
            robotMarkerDistance2D = float(math.hypot(float(t.x) - robotX, float(t.y) - robotY))
            robotMarkerBearingMap = float(math.atan2(float(t.y) - robotY, float(t.x) - robotX))
            # Eventually, we caould also compute the robot's yaw, but that was NOT used in the final version of the code
            # robotQ = self.lastAMCLMsg.pose.pose.orientation
            # robotYawMap = float(
            #     SciPyRotation.from_quat([
            #         float(robotQ.x),
            #         float(robotQ.y),
            #         float(robotQ.z),
            #         float(robotQ.w),
            #     ]).as_euler('xyz')[2]
            # )
        sample = ArucoSample(
            stamp_sec = int(msg.header.stamp.sec), # Seconds part of the ROS timestamp associated with the transform message.
            stamp_nanosec = int(msg.header.stamp.nanosec), # Nanoseconds part of the ROS timestamp associated with the transform message.
            frame_id = str(msg.header.frame_id), # Reference frame in which the marker pose is expressed, typically "map".
            child_frame_id = str(msg.child_frame_id), # TF child frame associated with the detected ArUco marker.
            position = [float(t.x), float(t.y), float(t.z)], # Marker position [x, y, z] expressed in frame_id, in meters.
            quaternion = [float(q.x / norm), float(q.y / norm), float(q.z / norm), float(q.w / norm)], # Normalized marker orientation [x, y, z, w] expressed in frame_id.
            robot_position = robotPosition, # Estimated robot position {"x": ..., "y": ...} from AMCL at detection time, if available.
            robot_marker_distance_2d = robotMarkerDistance2D, # Planar distance between the robot and the marker in the map frame, if AMCL is available.
            robot_marker_bearing_map = robotMarkerBearingMap, # Planar bearing angle from the robot to the marker in the map frame, in radians, if AMCL is available.
            # robot_yaw_map = robotYawMap
        )
        samples.append(sample)

    def startArucoCollection(self):
        # This method is called in the moment in which the discovery begins and the collection of the Aruco Marker location samples have to start
        self.pickArucoSamples.clear()
        self.placeArucoSamples.clear()
        self.collectArucoSamples = True
        self.arucoLocationsSaved = False
        self.logInfoWithStatus("Started collecting pick/place marker samples during discovery.")

    def computePositionsMedoid(self, samples):
        # This method computes the mediod position among the list of samples received in input
        # Note that the medioid is the one vector (among all the considered ones) that minimizes the sum of the distances from all the others
        positions = np.array([sample.position for sample in samples], dtype = float)
        pairwiseDistances = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
        medoidIndex = int(np.argmin(pairwiseDistances.sum(axis=1)))
        return positions[medoidIndex].tolist()

    def computeOrientationsMedoid(self, samples):
        # This method computes the mediod orientation (expressed as a quaternion) among the list of samples received in input
        # Note that it is selected the quaternion which rotation has the minimum total angular distance from all other sample quaternion orientations
        quaternions = np.array([sample.quaternion for sample in samples], dtype = float)
        rotations = SciPyRotation.from_quat(quaternions)
        totalDistances = []
        for i in range(len(rotations)):
            relativeRotations = rotations[i].inv() * rotations
            totalDistances.append(float(np.sum(relativeRotations.magnitude())))
        medoidIndex = int(np.argmin(totalDistances))
        return quaternions[medoidIndex].tolist()

    def buildLocationEstimate(self, samples, label: str, markerID: int, markerFrame: str):
        # This method builds the final estimate of the pick/place Aruco Marker location based on the collected samples
        if len(samples) == 0:
            self.logErrorWithStatus(f"No samples collected for {label} marker {markerID}. Returning an empty estimate.")
            return params.ArucoPoseEstimate(
                found = False, # Used only in case NO samples at all were collected (this should NOT happen if the discovery policy is a good one)
                marker_id = markerID,
                label = label,
                frame_id = params.fatherReferenceFrame,
                marker_frame = markerFrame,
                sample_count = 0,
                pose = None
            )
        # Filtering samples, preserving only ones that are sufficiently close to the Tiago (ad the moment of the detection)
        filteredSamples = []
        for sample in samples:
            distance = sample.robot_marker_distance_2d
            if distance is None:
                continue
            elif distance <= params.arucoSampleDistanceThresholdDuringDiscovery:
                filteredSamples.append(sample)
        if len(filteredSamples) == 0:
            self.logErrorWithStatus(f"All {label} marker {markerID} samples were filtered out because they were too far from the robot (distance threshold: {params.arucoSampleDistanceThresholdDuringDiscovery} m).")
            return params.ArucoPoseEstimate(
                found = False, # Used only in case NO samples at all were collected (this should NOT happen if the discovery policy is a good one)
                marker_id = markerID,
                label = label,
                frame_id = params.fatherReferenceFrame,
                marker_frame = markerFrame,
                sample_count = 0,
                pose = None
            )
        else:
            self.logInfoWithStatus(f"Collected {len(samples)} samples for {label} marker {markerID}, of which {len(filteredSamples)} were kept after filtering (distance threshold: {params.arucoSampleDistanceThresholdDuringDiscovery} m).")

        position = self.computePositionsMedoid(filteredSamples)
        quaternion = self.computeOrientationsMedoid(filteredSamples)
        frameId = filteredSamples[0].frame_id if filteredSamples[0].frame_id else params.fatherReferenceFrame
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = float(quaternion[0])
        pose.orientation.y = float(quaternion[1])
        pose.orientation.z = float(quaternion[2])
        pose.orientation.w = float(quaternion[3])
        return params.ArucoPoseEstimate(
            found = True, # Whether at least one valid marker sample was collected.
            marker_id = markerID, # Numerical ID of the ArUco marker associated with this location.
            label = label, # Semantic role of the location, e.g. "pick" or "place".
            frame_id = frameId, # Reference frame in which the estimated pose is expressed, typically "map" ("map" is indeed used as a default value).
            marker_frame = markerFrame, # TF frame name associated with the specific ArUco marker.
            sample_count = len(filteredSamples), # Number of samples used to compute the estimate.
            pose = pose # The estimated pose of the ArUco marker in the specified frame_id.
        )

    def saveLocationJSON(self, filename: str, data: dict):
        # This method saves the given data (expected to be in dictionary form) in JSON format and in the same folder where the map is stored
        outputPath = Path(self.savedMapPath) / filename
        outputPath.parent.mkdir(parents = True, exist_ok = True)
        with open(outputPath, 'w', encoding='utf-8') as f: json.dump(data, f, indent=2)
        self.logInfoWithStatus(f"Saved {filename} to: {outputPath}")

    def buildLocationHistoryData(self, samples, label: str, marker_id: int, marker_frame: str):
        # This method builds a dictionary containing the history of the collected samples (used both for pick and place locations) to be later on saved in JSON format
        # This information are not gonna be strictly foundamental (only the final single estimate about each marker pose is really needed), but they are saved anyway for debugging and analysis purposes
        return {
            'marker_id': marker_id,
            'label': label,
            'frame_id': samples[0].frame_id if len(samples) > 0 and samples[0].frame_id else params.fatherReferenceFrame,
            'marker_frame': marker_frame,
            'sample_count': len(samples),
            'frames': [asdict(sample) for sample in samples],
        }

    def saveArucoLocations(self):
        # This method exploits all the previous methods to compute the final estimate of the pick/place Aruco Marker locations and save in JSON format all the related data 
        self.collectArucoSamples = False
        pickData = self.buildLocationEstimate(
            self.pickArucoSamples,
            label = params.pickLocationMarker.markerNickname,
            markerID = params.pickLocationMarker.markerID,
            markerFrame = params.pickLocationMarker.markerFrame
        )
        placeData = self.buildLocationEstimate(
            self.placeArucoSamples,
            label = params.placeLocationMarker.markerNickname,
            markerID = params.placeLocationMarker.markerID,
            markerFrame = params.placeLocationMarker.markerFrame
        )
        pickHistoryData = self.buildLocationHistoryData(
            self.pickArucoSamples,
            label = params.pickLocationMarker.markerNickname,
            marker_id = params.pickLocationMarker.markerID,
            marker_frame = params.pickLocationMarker.markerFrame
        )
        placeHistoryData = self.buildLocationHistoryData(
            self.placeArucoSamples,
            label = params.placeLocationMarker.markerNickname,
            marker_id = params.placeLocationMarker.markerID,
            marker_frame = params.placeLocationMarker.markerFrame
        )
        self.saveLocationJSON(params.pickLocationJSONFileName, pickData.toDict())
        self.saveLocationJSON(params.placeLocationJSONFileName, placeData.toDict())
        self.saveLocationJSON(params.pickLocationHistoryJSONFileName, pickHistoryData)
        self.saveLocationJSON(params.placeLocationHistoryJSONFileName, placeHistoryData)
        self.arucoLocationsSaved = True
        self.logInfoWithStatus(f"Final samples: pick={len(self.pickArucoSamples)}, place={len(self.placeArucoSamples)}.")

    def stepUpFSM(self):
        if self.currentFSMstate == Task2FSMState.TUCK_ARM: self.handle_tuckArm()
        elif self.currentFSMstate == Task2FSMState.WAIT_ARM_TUCKED: self.handle_waitArmTucked()
        elif self.currentFSMstate == Task2FSMState.TRIGGER_LOCALIZATION_RESTART: self.handle_triggerLocalizationRestart()
        elif self.currentFSMstate == Task2FSMState.WAIT_LOCALIZATION_RESTARTED: self.handle_waitLocalizationRestarted()
        elif self.currentFSMstate == Task2FSMState.EVALUATING_LOCALIZATION: self.handle_evaluatingLocalization()
        elif self.currentFSMstate == Task2FSMState.LOCALIZING: self.handle_localizing()
        elif self.currentFSMstate == Task2FSMState.SET_HEAD_FOR_DISCOVERY: self.handle_setHeadForDiscovery()
        elif self.currentFSMstate == Task2FSMState.WAIT_HEAD_FOR_DISCOVERY: self.handle_waitHeadForDiscovery()
        elif self.currentFSMstate == Task2FSMState.DISCOVERING: self.handle_discovering()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_SET_YAW_TOLERANCE: self.handle_discoverySetYawTolerance()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_WAIT_YAW_TOLERANCE_SET: self.handle_discoveryWaitYawToleranceSet()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_COMPUTE_PATH: self.handle_discoveryComputePath()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_WAIT_PATH: self.handle_discoveryWaitPath()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_SPIN_TO_PATH_HEADING: self.handle_discoverySpinToPathHeading()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_WAIT_SPIN: self.handle_discoveryWaitSpin()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_NAVIGATE_TO_GOAL: self.handle_discoveryNavigateToGoal()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_WAIT_NAVIGATION: self.handle_discoveryWaitNavigation()    
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_RESTORE_YAW_TOLERANCE: self.handle_discoveryRestoreYawTolerance()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_WAIT_YAW_TOLERANCE_RESTORED: self.handle_discoveryWaitYawToleranceRestored()
        elif self.currentFSMstate == Task2FSMState.DISCOVERY_FINISHED: self.handle_discoveryFinished()
        else: self.get_logger().error(f"Encountered an unknown FSM state: {self.currentFSMstate}")

    # +------------------------------------------------------------------------------------------------------------------------+
    # | The following portion of code related to the tuck of the arm is exactly the same as the one used in the Task1 FSM Node |
    # +------------------------------------------------------------------------------------------------------------------------+

    def handle_tuckArm(self):
        if not self.tiagoArmActionClient.server_is_ready(): # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
            self.logInfoWithStatus("Waiting for TiagoArm Action Server...")
            return
        self.logInfoWithStatus("Sending goal to TiagoArm Action Server...")
        goalMsg = TiagoArm.Goal()
        goalMsg.joint_positions = params.HOME_JOINT_POSITIONS
        self.armTuckGoalDone = False
        self.armTuckGoalSucceeded = False
        sendGoalFuture = self.tiagoArmActionClient.send_goal_async(
            goalMsg,
            feedback_callback = self.armFeedbackCallback
        )
        sendGoalFuture.add_done_callback(self.armGoalResponseCallback)
        self.currentFSMstate = Task2FSMState.WAIT_ARM_TUCKED

    def armFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.logInfoWithStatus(f"Feedback: {feedback.current_state}")

    def armGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if not goalHandle.accepted:
            self.logErrorWithStatus("Goal rejected by TiagoArm Action Server.")
            self.armTuckGoalDone = True
            self.armTuckGoalSucceeded = False
            return
        self.logInfoWithStatus("Goal accepted by TiagoArm Action Server.")
        getResultFuture = goalHandle.get_result_async()
        getResultFuture.add_done_callback(self.armResultCallback)

    def armResultCallback(self, future):
        result = future.result().result # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if result.success:
            self.logInfoWithStatus(f"Success: {result.message}")
            self.armTuckGoalDone = True
            self.armTuckGoalSucceeded = True
        else:
            self.logErrorWithStatus(f"Failed: {result.message}")
            self.armTuckGoalDone = True
            self.armTuckGoalSucceeded = False

    def handle_waitArmTucked(self):
        self.logInfoWithStatus("Waiting for arm to be tucked...")
        # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
        if self.armTuckGoalDone:
            if self.armTuckGoalSucceeded:
                self.logInfoWithStatus("Arm motion completed.")
                self.currentFSMstate = Task2FSMState.TRIGGER_LOCALIZATION_RESTART
            else:
                self.logWarnWithStatus("Arm goal failed or was rejected by TiagoArm Action Server. Retrying...")
                self.currentFSMstate = Task2FSMState.TUCK_ARM

    # +------------------------------------------------------------------------------------------------------------------------+
    # |                            End of the part fo the code related to the tuck of the arm                                  |
    # +------------------------------------------------------------------------------------------------------------------------+

    # +------------------------------------------------------------------------------------------------------------------------+
    # |                                               Autonomous Localization                                                  |
    # +------------------------------------------------------------------------------------------------------------------------+

    def isAMCLready(self) -> bool:
        # This method checks whether the AMCL/localization stack is ready to be used (i.e. it is operationally ready)
        # Note that this check is performed by verifying that:
        # (1) there is at least one publisher on the "amcl_pose" topic and
        # (2) the "reinitialize_global_localization" service is ready to be called
        amclPosePublishersCount = self.count_publishers(params.amclPoseTopic)
        reinitGlobLocServiceReady = self.reinitializeGlobalLocalizationClient.service_is_ready()
        self.logInfoWithStatus(
            f"amcl_pose publishers: {amclPosePublishersCount}, "
            f"reinitialize_global_localization service ready: {reinitGlobLocServiceReady}"
        )
        return amclPosePublishersCount > 0 and reinitGlobLocServiceReady

    def handle_triggerLocalizationRestart(self):
        # In this state, the FSM triggers a full fresh restart of the autonomous localization procedure
        if not self.isAMCLready():
            self.logInfoWithStatus("Now waiting for AMCL/localization Stack readiness...")
            return # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
        self.logInfoWithStatus("AMCL/localization Stack is operationally ready!")
        self.logInfoWithStatus("Calling reinitialize_global_localization service (the aim is to start from a fresh re-initialized localization procedure)...")
        self.globalLocalizationResetDone = False
        self.globalLocalizationResetSucceeded = False
        req = Empty.Request()
        future = self.reinitializeGlobalLocalizationClient.call_async(req)
        future.add_done_callback(self.reinitializeGlobalLocalizationCallback)
        self.currentFSMstate = Task2FSMState.WAIT_LOCALIZATION_RESTARTED

    def reinitializeGlobalLocalizationCallback(self, future):
        self.globalLocalizationResetDone = True
        try:
            _ = future.result() # As indicated in the documentation (note that this is an Empty service)
            self.globalLocalizationResetSucceeded = True
        except Exception as e:
            self.globalLocalizationResetSucceeded = False
            self.logErrorWithStatus(f"Global localization reset request failed, error was: {e}")

    def handle_waitLocalizationRestarted(self):
        # In this state, the FSM waits for the completion of the global localization reset procedure (and eventually restart it in case of failure)
        if self.globalLocalizationResetDone:
            if self.globalLocalizationResetSucceeded:
                self.logInfoWithStatus("\n |>\n |> Global localization reset completed successfully!\n |>")
                self.currentFSMstate = Task2FSMState.EVALUATING_LOCALIZATION
            else:
                self.logErrorWithStatus("n |>\n |> Global localization reset failed, now retrying...\n |>")
                self.currentFSMstate = Task2FSMState.TRIGGER_LOCALIZATION_RESTART
    
    def updateLocalizationMetricsHistory(self, metric):
        # This method enrich the history of the localization metrics.
        # Indeed, the localization metric is periodically computed and stored (as it will be shown in the following code).
        # Note that the history of the localization metric has a limited lenght: when the maximum length is reached, the oldest metric is removed from the history!
        self.localizationMetricsHistory.append(float(metric))
        if len(self.localizationMetricsHistory) > self.localizationMetricsWindowSize:
            self.localizationMetricsHistory.pop(0)

    def isLocalizationMetricImprovingEnough(self):
        # This method checks whether the localization metric is improving enough (i.e. it is decreasing enough) or not.
        # The basic idea behind this check is: if the localization metric is not improving enough, then some action should be taken in relation to that.
        # The check is carried out accordingly to the following procedure:
        # (1) if the history of the localization metric is still not full, then True is immediatly returned
        # (2) if the current metric satisfies the threshold of being considered "good enough", then True is immediatly returned
        # (3) if the current metric is greater than [or equal to] value 1.0 (that is, it's considered still quite big),
        #     then it is checked whether the current metric has jumped up with respect to the previous one.
        #     If that's the case (indeed, a big jump has been detected), it's assumed that the localization procedure has just performed a big correction,
        #     so the evaluation of its improvement is reset (and True is returned).
        # (4) if the oldest metric is less than or equal to 0.0 (should not happen), then it is assumed that the metric is improving enough (i.e. return True)
        # (5) Finally, the improvement percentage is computed and compared with the related threshold (returning True or False as a conseguence).
        if len(self.localizationMetricsHistory) < self.localizationMetricsWindowSize: return True
        previousMetric = self.localizationMetricsHistory[-2]
        currentMetric = self.localizationMetricsHistory[-1]
        if currentMetric < params.autonomousLocalizationMetricsTreshold: return True
        if currentMetric >= 1.0 and previousMetric > 0.0:
            jumpUpPercent = ((currentMetric - previousMetric) / previousMetric) * 100.0
            if jumpUpPercent >= self.localizationMetricsJumpUpResetPercent:
                self.logWarnWithStatus(f"\n |>\n |> Large metric jump detected ({jumpUpPercent:.2f}%). Resetting localization metrics history.\n |>")
                self.localizationMetricsHistory.clear()
                return True
        oldestMetric = self.localizationMetricsHistory[0]
        newestMetric = self.localizationMetricsHistory[-1]
        if oldestMetric <= 0.0: return True # This is just a safety check to avoid division by zero or negative values (it should never happen in practice)
        improvementPercent = ((oldestMetric - newestMetric) / oldestMetric) * 100.0
        return improvementPercent >= self.localizationMetricsMinImprovementPercent
    
    def collisionMonitorCallback(self, future):
        # This method is the callback for the Collision Monitor Service response to a request for enabling OR disabling it.
        # Note that this callback has been realized taking as reference the related documentation of Ros2 Humble
        try:
            response = future.result()
            if response is None:
                self.logErrorWithStatus("No response received from Collision Monitor (unexpected)! This will be ignored")
                return
            result = response.results[0]
            if result.successful: self.logInfoWithStatus("Collision Monitor correctly updated!")
            else: self.logErrorWithStatus(f"Failed: {result.reason}")
        except Exception as e: self.logErrorWithStatus(f"Exception in collision monitor callback (unexpected)(this will be ignored): {e}")
        finally:
            # TODO: manage in a better way the situation in wwhich the Collision Monitor response is an unexpected one
            self.logInfoWithStatus("\n |>\n |> Collision monitor callback completed\n |>")
            self.collisionMonitorResponseReceived = True
            
    def handle_evaluatingLocalization(self):
        # In this state, the FSM evaluates the quality of the localization process by checking the covariance of the AMCL pose estimation.
        # On the basis of that evaluation, it takes different actions.
        # The implemented behavior is the following: the autonomous localization is performed through a simple rotation in-place.
        # The rotation in-place is performed by the robot until the localization quality is good enough (i.e. the covariance of the AMCL pose estimation is below a certain threshold).
        # At the same time, it is also periodically checked whether the localization quality is improving or not.
        # In case no improvement is detected, then the current position is considered unsuitable to be used for an autonomous localization performed via in-place rotation.
        # In that case, a simple DriveOnHeading Action is performed to reach a new position (protected with the usage of a Collision Monitor), then the autonomous localization procedure is restarted from scratch (i.e. the AMCL pose estimation is reinitialized and the in-place rotation is performed again).
        self.logInfoWithStatus("Evaluating localization quality...")
        if self.lastAMCLMsg is None: self.logInfoWithStatus("Still waiting for AMCL first incoming data (no data at all received yet)...")
        else:
            message = self.lastAMCLMsg
            covariance = message.pose.covariance
            xVariance = covariance[0]
            yVariance = covariance[7]
            yawVariance = covariance[35]
            metric = xVariance + yVariance + yawVariance # This is the metric used for the evaluation of the localization quality (the lower, the better)!
            self.updateLocalizationMetricsHistory(metric)
            self.logInfoWithStatus(f"\n |>\n |> Current AMCL pose covariance metric: {metric} (xVar: {xVariance}, yVar: {yVariance}, yawVar: {yawVariance})\n |>")
            if metric < params.autonomousLocalizationMetricsTreshold:
                self.localizationGoodMetricConsecutiveCount += 1
                self.localizationMetricsHistory.clear()
                self.logInfoWithStatus(f"\n |>\n |> Localization quality is good enough (metric: {metric}) for {self.localizationGoodMetricConsecutiveCount} consecutive evaluations (required a total of {self.localizationGoodMetricConsecutiveCountRequired}).\n |>")
                if self.localizationGoodMetricConsecutiveCount >= self.localizationGoodMetricConsecutiveCountRequired:
                    self.logInfoWithStatus(f"\n |>\n |> Autonomous localization has been correctly fullfilled (final metric: {metric}).\n |>")
                    x = message.pose.pose.position.x
                    y = message.pose.pose.position.y
                    self.robotXY = (x, y)
                    self.currentFSMstate = Task2FSMState.SET_HEAD_FOR_DISCOVERY
                    return
            else: self.localizationGoodMetricConsecutiveCount = 0
            if not self.spinActionClient.server_is_ready() or not self.driveOnHeadingActionClient.server_is_ready() or not self.collisionMonitorEnabler.service_is_ready():
                self.logWarnWithStatus("Localization quality is not good enough, now waiting for the Spin ActionServer, the DriveonHeading ActionServer (of the Nav2 Stack) and the Collision Monitor Service to be ready...")
            else:
                # The localization quality is not good enough, so the FSM will trigger a proper procedure to improve it!
                # A Spin and a DriveOnHeading couple of Actions are periodically alternated.
                # The "self.localizationPhase" variable is used to keep track of which Action has to be triggered NEXT (0 for Spin, 1 for DriveOnHeading).
                # That is, if on zero, then the last performed Action was a DriveOnHeading.
                # Otherwise, if on one, then the last performed Action was a Spin.
                self.logWarnWithStatus("Localization quality is not good enough, now triggering improvement procedure...")
                if self.localizationPhase is None: self.localizationPhase = 0
                self.localizationSpinDone = False
                self.localizationSpinSucceeded = False
                self.localizationDriveOnHeadingDone = False
                self.localizationDriveOnHeadingSucceeded = False
                # self.localizationGoodMetricConsecutiveCount = 0
                
                if self.localizationPhase == 0: # That is, we need to pass to a Spin Action (the last performed Action was a DriveOnHeading)
                    if self.collisionMonitorEnabled: # The Collision Monitor should NOT be enabled during the Spin Action
                        self.localizationGoodMetricConsecutiveCount -= 1 # We are gonna repeat this handler only to wait for the Collision Monitor to be disabled,
                        # so we need to decrement the count of the consecutive good metrics (otherwise, it would be incremented again in the next iteration)
                        if not self.collisionMonitorRequestSent:
                            # Accordingly to the Collision Monitor Ros2 Humble documentation, the following lines of code disable the Collision Monitor itself
                            req = SetParameters.Request()
                            param = Parameter('PolygonStop.enabled', value = False)
                            req.parameters = [param.to_parameter_msg()]
                            future = self.collisionMonitorEnabler.call_async(req)
                            future.add_done_callback(self.collisionMonitorCallback)
                            self.collisionMonitorResponseReceived = False
                            self.collisionMonitorRequestSent = True
                        else:
                            if self.collisionMonitorResponseReceived:
                                self.collisionMonitorRequestSent = False
                                self.collisionMonitorEnabled = False
                    else:
                        # Now executing the Spin Action (the Collision Monitor is disabled, as it should be in this phase)
                        self.collisionMonitorRequestSent = False
                        self.collisionMonitorResponseReceived = False
                        goalMsg = Spin.Goal()
                        # In a previous version of the code, I was also using a random-walk approach (UNUSED in the final version of the code: RANDOMWALKlocalization = False)
                        orientationDirection = random.choice([-1, 1]) # The spin orientation is randomly choosen ONLY in case of a random-walk approach
                        if not self.RANDOMWALKlocalization: orientationDirection = 1
                        goalMsg.target_yaw = orientationDirection*params.autonomousLocalizationSpinYaw
                        goalMsg.time_allowance = Duration(sec = int(params.autonomousLocalizationSpinTimeAllowance))
                        sendGoalFuture = self.spinActionClient.send_goal_async(
                            goalMsg,
                            feedback_callback = self.localizationSpinFeedbackCallback
                        )
                        sendGoalFuture.add_done_callback(self.localizationSpinGoalResponseCallback)
                        # In a previous version of the code, I was also using a random-walk approach (UNUSED in the final version of the code: RANDOMWALKlocalization = False)
                        # If the localization metric is not improving enough, then it means that we are in a position in which a simple in-place rotation is not sufficient to successfully improve the localization quality: we need to move!
                        if not self.isLocalizationMetricImprovingEnough() or self.RANDOMWALKlocalization: self.localizationPhase = 1

                elif self.localizationPhase == 1: # That is, we need to pass to a DriveOnHeading Action (the last performed Action was a Spin)
                    if not self.collisionMonitorEnabled: # The Collision Monitor should be enabled during the DriveOnHeading Action
                        self.localizationGoodMetricConsecutiveCount -= 1 # We are gonna repeat this handler only to wait for the Collision Monitor to be enabled,
                        # so we need to decrement the count of the consecutive good metrics (otherwise, it would be incremented again in the next iteration)
                        if not self.collisionMonitorRequestSent:
                            # Accordingly to the Collision Monitor Ros2 Humble documentation, the following lines of code enable the Collision Monitor itself
                            req = SetParameters.Request()
                            param = Parameter('PolygonStop.enabled', value = True)
                            req.parameters = [param.to_parameter_msg()]
                            future = self.collisionMonitorEnabler.call_async(req)
                            future.add_done_callback(self.collisionMonitorCallback)
                            self.collisionMonitorResponseReceived = False
                            self.collisionMonitorRequestSent = True
                        else:
                            if self.collisionMonitorResponseReceived:
                                self.collisionMonitorRequestSent = False
                                self.collisionMonitorEnabled = True
                    else:
                        # Now executing the DriveOnHeading Action (the Collision Monitor is enabled, as it should be in this phase)
                        self.collisionMonitorRequestSent = False
                        self.collisionMonitorResponseReceived = False
                        goalMsg = DriveOnHeading.Goal()
                        goalMsg.target.x = params.autonomousLocalizationDriveDistance
                        goalMsg.target.y = 0.0
                        goalMsg.target.z = 0.0
                        goalMsg.speed = float(params.autonomousLocalizationDriveSpeed)
                        goalMsg.time_allowance = Duration(sec = int(params.autonomousLocalizationDriveTimeAllowance))
                        sendGoalFuture = self.driveOnHeadingActionClient.send_goal_async(
                            goalMsg,
                            feedback_callback = self.driveOnHeadingFeedbackCallback
                        )
                        sendGoalFuture.add_done_callback(self.driveOnHeadingGoalResponseCallback)
                        # In a previous version of the code, I was also using a random-walk approach (UNUSED in the final version of the code: RANDOMWALKlocalization = False)
                        # Due to the fact that the localization metric was not improving enough, a DriveOnHeading Action has been triggered.
                        # After that, the autonomous localization procedure (AKA in-place rotation) will be restarted from scratch from the new reaced position.
                        # That is, the localization metric hystory must be reset (and the FSM state "TRIGGER_LOCALIZATION_RESTART" will be reached)
                        if not self.RANDOMWALKlocalization: self.localizationMetricsHistory.clear()
                        self.localizationPhase = 0

                if not self.collisionMonitorRequestSent and not self.collisionMonitorResponseReceived: self.currentFSMstate = Task2FSMState.LOCALIZING
    
    def localizationSpinFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        angle_rad = feedback.angular_distance_traveled
        angle_deg = angle_rad * 180.0 / 3.14
        # self.logInfoWithStatus(f"Spin in progress...  rotated so far: {angle_rad:.3f} rad ({angle_deg:.1f} deg)")
        # Lines of code commented out to avoid too verbose logging

    def driveOnHeadingFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.logInfoWithStatus(f"DriveOnHeading in progress...  distance traveled so far: {feedback.distance_traveled:.3f} m")
        # Lines of code commented out to avoid too verbose logging

    def localizationSpinGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if not goalHandle.accepted:
            self.logErrorWithStatus("Spin goal rejected by Nav2.")
            self.localizationSpinDone = True
            self.localizationSpinSucceeded = False
            return
        self.logInfoWithStatus("Spin goal accepted by Nav2.")
        getResultFuture = goalHandle.get_result_async()
        getResultFuture.add_done_callback(self.localizationSpinResultCallback)

    def driveOnHeadingGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if not goalHandle.accepted:
            self.logErrorWithStatus("DriveOnHeading goal rejected by Nav2.")
            self.localizationDriveOnHeadingDone = True
            self.localizationDriveOnHeadingSucceeded = False
            return
        self.logInfoWithStatus("DriveOnHeading goal accepted by Nav2.")
        getResultFuture = goalHandle.get_result_async()
        getResultFuture.add_done_callback(self.driveOnHeadingResultCallback)

    def localizationSpinResultCallback(self, future):
        result = future.result().result # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        elapsed = result.total_elapsed_time.sec + result.total_elapsed_time.nanosec * 1e-9
        self.logInfoWithStatus(f"Spin behavior completed (with status '{future.result().status}', result '{future.result().result}' and total_elapsed_time '{elapsed}').")
        self.localizationSpinDone = True
        self.localizationSpinSucceeded = (future.result().status == GoalStatus.STATUS_SUCCEEDED)

    def driveOnHeadingResultCallback(self, future):
        result = future.result().result # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        elapsed = result.total_elapsed_time.sec + result.total_elapsed_time.nanosec * 1e-9
        self.logInfoWithStatus(f"DriveOnHeading behavior completed (with status '{future.result().status}', result '{future.result().result}' and total_elapsed_time '{elapsed}').")
        self.localizationDriveOnHeadingDone = True
        self.localizationDriveOnHeadingSucceeded = (future.result().status == GoalStatus.STATUS_SUCCEEDED)
        
    def handle_localizing(self):
        # self.get_logger().info("[LOCALIZING] Waiting for the localization movement to be completed...")
        # Lines of code commented out to avoid too verbose logging
        if self.localizationSpinDone:
            if self.localizationSpinSucceeded:
                self.logInfoWithStatus("Spin movement completed successfully, now going back to evaluate localization quality...")
                self.currentFSMstate = Task2FSMState.EVALUATING_LOCALIZATION
            else:
                self.logErrorWithStatus("Spin movement failed... anyway, now going back to evaluate localization quality and deciding the next step...")
                self.currentFSMstate = Task2FSMState.EVALUATING_LOCALIZATION
        if self.localizationDriveOnHeadingDone:
            if not self.RANDOMWALKlocalization:
                # Note this important behaviour: in case of NOT relying on a random-walk-like approach (as it IS in the final version of the code),
                # then after each single DriveOnHeading Action, the FSM state is set to "TRIGGER_LOCALIZATION_RESTART".
                # This is exactly as expected and commented out before: after the position change, the autonomous localization procedure is restarted from scratch.
                self.currentFSMstate = Task2FSMState.TRIGGER_LOCALIZATION_RESTART
            elif self.localizationDriveOnHeadingSucceeded:
                self.logInfoWithStatus("DriveOnHeading movement completed successfully, now going back to evaluate localization quality...")
                self.currentFSMstate = Task2FSMState.EVALUATING_LOCALIZATION
            else:
                self.logErrorWithStatus("DriveOnHeading movement failed... anyway, now going back to evaluate localization quality and deciding the next step...")
                self.currentFSMstate = Task2FSMState.EVALUATING_LOCALIZATION

    # +------------------------------------------------------------------------------------------------------------------------+
    # |                                          Pick & Place Locations Discovery                                              |
    # +------------------------------------------------------------------------------------------------------------------------+

    def getJointPosition(self, jointName):
        # A very simple method that exploit the last available JointState message to extract the current position of a given joint
        # If no info is available yet, or if it's not retrievable, then None is returned
        if self.latestJointState is None: return None
        try:
            idx = self.latestJointState.name.index(jointName)
            return float(self.latestJointState.position[idx])
        except (ValueError, IndexError): return None

    def publishHeadTiltCommand(self):
        # This method can be used to publish a command to control the tilt-position of the Tiago head
        # Very important: note that the Publiser for the Tiago head tilt must be used in a very precise manner. This is indicated in the related documentation!
        # It always requires to publish a command for BOTH the pan of the head (rotation around the vertical axis)
        # AND the tilt of the head (rotation around the horizontal axis)!
        # The pan of the head is associated to the name "head_1_joint" and the tilt of the head is associated to the name "head_2_joint".
        # That is, if we want to only alter the tilt, we must read the current pan position and explicitly publish it together with the new tilt position.
        current_head_pan = self.getJointPosition(params.panHeadJointName)
        if current_head_pan is None:
            self.logWarnWithStatus("Cannot read current " + params.panHeadJointName + " yet from topic " + params.jointStateTopic + ", retrying...")
            return False
        target_head_tilt = float(params.headTiltDuringDiscovery)
        msg = JointTrajectory()
        msg.joint_names = [params.panHeadJointName, params.tiltHeadJointName]
        point = JointTrajectoryPoint()
        point.positions = [current_head_pan, target_head_tilt]
        point.time_from_start = Duration(sec = 2) # Imposed time duration for the head movement
        msg.points.append(point)
        self.headCommandPublisher.publish(msg)
        self.logInfoWithStatus(f"Published head command: {params.panHeadJointName}={current_head_pan:.3f}, {params.tiltHeadJointName}={target_head_tilt:.3f}.")
        return True

    def handle_setHeadForDiscovery(self):
        # In this state, once information about the current head pan is available, the FSM published a command to move to a specific head tilt value
        # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
        if self.publishHeadTiltCommand(): self.currentFSMstate = Task2FSMState.WAIT_HEAD_FOR_DISCOVERY

    def handle_waitHeadForDiscovery(self):
        # In this state, the FSM waits for the head tilt to reach the desired value
        current_head_tilt = self.getJointPosition(params.tiltHeadJointName)
        if current_head_tilt is None:
            self.logInfoWithStatus("Waiting for a value for " + params.tiltHeadJointName + " from " + params.jointStateTopic + "...")
            return
        target_head_tilt = float(params.headTiltDuringDiscovery)
        head_tilt_error = abs(current_head_tilt - target_head_tilt)
        if head_tilt_error <= self.headTiltTolerance:
            self.logInfoWithStatus(f"Head tilt reached: current={current_head_tilt:.3f}, target={target_head_tilt:.3f}, error={head_tilt_error:.3f}.")
            self.currentFSMstate = Task2FSMState.DISCOVERING
        else: self.logInfoWithStatus(f"Waiting head tilt: current={current_head_tilt:.3f}, target={target_head_tilt:.3f}, error={head_tilt_error:.3f}.")

    def handle_discovering(self):
        # In this state, the FSM starts the discovery procedure.
        # Of course, this can be done only AFTER the autonomous localization has been successfully completed AND the Tiago head has been properly tilted.
        # Given the map of the environment and the current robot position within it, a proper Discovery Planner is exploited to set-up the discovery phase.
        # A set of keypoints are returned by the Discovery Planner: the main idea is that these is an exhaustive set of significant point all around the map such that
        # if the Tiago robot moves to all of them, and for each one of them performs a full in-place rotation, then ALL the envirnoment will be successfully observed
        # by the robot sensors AKA the pick and place locations are gonna be surely discovered.
        # To fasten up the procedure, in each keypoint a full in-place rotation is avoided: the robot is simply gonna to re-orient itself toward the next keypoint,
        # choosing to perform (among the two possibile ones) the longest spin/rotation for that re-orientation. This is assumed to be sufficient for a full discovery.
        #
        # An important note: how the keyponts are ordered (i.e. the order in which the robot is gonna to visit them) is quite relevant to fasten up thhis discovery phase.
        # In this sense, the Discovery Planner is gonna return the keypoints as ALREADY ordered for the discovery.
        # In a previous version of the code, the Task2 FSM was also implementing a custom re-ordering procedure that was exploiting the ComputePathToPose Action Server
        # to evaluate the cost (in terms of path-distance) to reach each candidate keypoint and decide the next one to visit on the basis of that. Unfortunately, this
        # approach proved itself to be too heavy (the amount of calls to the ComputePathToPose Action Server was quite high and the time required to execute all of them
        # was A LOT); conseguently, in the final version of the code, this custom re-ordering procedure has been totally removed.

        self.startArucoCollection() # For this moment, the collection of the observed Aruco Markers is started (as it should of course be during the discovery)
        self.logInfoWithStatus("Computing ordered discovery keypoints...")

        keypoints, goals, firstSpuriousIndex = discoveryPlanner.computeOrderedKeypoints( # Exploit the Discovery Planner to compute the ordered keypoints set
            yamlPath = os.path.join(self.savedMapPath, self.savedMapName + ".yaml"),
            robotXYlocation = self.robotXY,
            visualize = True
        )
        self.logInfoWithStatus(f"Discovery planner returned {len(goals)} goals (first_spurious_index = {firstSpuriousIndex}).")

        self.discoveryOrderedKeypoints = list(keypoints)
        self.discoveryOrderedGoals = list(goals)
        if len(self.discoveryOrderedGoals) == 0: self.logErrorWithStatus("No discovery goals were produced by discoveryPlanner (unexpected). Returning to start only.")
        self.discoveryOrderedGoals.append(self.createReturnToStartGoal()) # Adding as a final keypoint the starting position of the robot (before starting the discovery procedure)
        self.discoveryCurrentGoalIndex = 0
        self.currentFSMstate = Task2FSMState.DISCOVERY_SET_YAW_TOLERANCE

    def normalizeAngle(self, angle):
        # A simple utiliy method to normalize an angle in radians to the range [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))

    def getCurrentYawFromAMCLmessages(self):
        # A simple method that exploits the last available AMCL message to extract the current yaw of the robot in the map frame (from quaternion to euler angle)
        # Note that this method is used at a stage of the FSM at which at least one AMCL message has already been received (so self.lastAMCLMsg can be supposed not None)
        q = self.lastAMCLMsg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def extractInitialPathHeading(self, path, min_segment_length = 0.05):
        # Given in input a global path, this method extracts the heading of the path.
        # Note that the "path" variable is of type "nav_msgs.msg.Path".
        # The heading is the angular direction from which the path starts (e.g. 0 heading towards X, pi/2 heading towards Y, pi heading towards -X, -pi/2 heading towards -Y).
        # Note that the heading is computed considering the very first point of the path AND the first next point that is at least at "min_segment_length" distance from it.
        if len(path.poses) < 2: return None
        x0 = path.poses[0].pose.position.x
        y0 = path.poses[0].pose.position.y
        for i in range(1, len(path.poses)):
            xi = path.poses[i].pose.position.x
            yi = path.poses[i].pose.position.y
            dx = xi - x0
            dy = yi - y0
            if math.hypot(dx, dy) > min_segment_length: return math.atan2(dy, dx)
        return None

    def computeLongSpin(self, currentYAW, targetYAW):
        # This methods takes as input a couple of angular positions (YAWs) and computes the longest spin/rotation to get from the first to the second.
        shortYAW = self.normalizeAngle(targetYAW - currentYAW) # The angular yaw difference is reduced to the range [-pi, pi]
        if shortYAW >= 0.0: return shortYAW - 2.0 * math.pi
        else: return shortYAW + 2.0 * math.pi

    def buildPoseFromXYLocation(self, xy):
        # This method simply builds a "geometry_msgs.msg.PoseStamped" pose message starting from a couple of (x, y) coordinates.
        # This is used to define the pose corresponding to a certain in-plane location in the map/environment.
        # IMPORTANT: the pose is built with a fixed orientation and referenced w.r.t. the "map" frame
        pose = NavigateToPose.Goal().pose
        pose.header.frame_id = params.fatherReferenceFrame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(xy[0])
        pose.pose.position.y = float(xy[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0 # Orientaiton facing towards +X of the map
        return pose

    def createReturnToStartGoal(self):
        returnGoal = NavigateToPose.Goal()
        returnGoal.pose = self.buildPoseFromXYLocation(self.robotXY)
        return returnGoal

    def handle_discoverySetYawTolerance(self):
        # In this state, before starting the discovery phase, the FSM alters the YAW tolerance of Nav2 Goal Checker to practically disable it (it is set to 360 degrees).
        # More specifically, firstly a GetParameters request exploited to read the actual value of the YAW tolerance parameter (this will be needed later on to restore the original value), then a SetParameters request is sent to update the YAW tolerance to 360 deg / 6.283185307179586 rad.
        # This is done beacuse ONLY during the discovery phase, the single Goal/Pose (sent to the Nav2 Stack) should be reached only in terms of location and not orientation.
        # Also note that: if the GetParameters fails, or also if the SetParameters fails, then the Goal Checker parameters are left unaltered: the discovery procedure will start anyway, but the final yaw (at each single keypoint) may still be enforced/controlled by Nav2.
        if not self.controllerServerGetParametersClient.service_is_ready():
            self.logInfoWithStatus(f"Waiting for {params.controllerServerGetParametersServiceName} service...")
            return
        if not self.controllerServerSetParametersClient.service_is_ready():
            self.logInfoWithStatus(f"Waiting for {params.controllerServerSetParametersServiceName} service...")
            return
        self.discoveryYawParamDone = False
        self.discoveryYawParamSucceeded = False
        # Building up a request for getting the current value of the YAW tolerance parameter from the controller_server Goal Checker
        req = GetParameters.Request()
        req.names = [self.yawGoalToleranceParamName]
        future = self.controllerServerGetParametersClient.call_async(req)
        future.add_done_callback(self.discoveryGetOriginalYawToleranceCallback)
        self.currentFSMstate = Task2FSMState.DISCOVERY_WAIT_YAW_TOLERANCE_SET

    def discoveryGetOriginalYawToleranceCallback(self, future):
        # In this callback, IFF the GetParameters request receives a success response, then a SetParameters one is sent.
        try:
            response = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
            if len(response.values) != 1 or response.values[0].type == 0:
                # If this IF in entered, then the parameter value was NOT properly retrieved.
                self.originalYawGoalTolerance = None
                self.discoveryYawParamDone = True
                self.discoveryYawParamSucceeded = False
                self.logErrorWithStatus(
                    f"Could not read parameter '{self.yawGoalToleranceParamName}' from {params.controllerServerGetParametersServiceName}. "
                    f"The runtime update to 360 deg / 2pi rad will NOT be applied! "
                    f"Discovery will continue anyway, but final yaw (at each single keypoint) may still be enforced/controlled by Nav2."
                )
                return
            # Saving the original value of the YAW tolerance parameter of the Goal Checker (to be later on used to be restored)
            self.originalYawGoalTolerance = response.values[0].double_value
            # Building up a request for setting the YAW tolerance parameter of the Goal Checker to 360 deg / 2pi rad
            req = SetParameters.Request()
            param = Parameter(
                self.yawGoalToleranceParamName,
                Parameter.Type.DOUBLE,
                2*math.pi # 6.283185307179586
            )
            req.parameters = [param.to_parameter_msg()]
            setFuture = self.controllerServerSetParametersClient.call_async(req)
            setFuture.add_done_callback(self.discoverySetYawToleranceCallback)
        except Exception as e:
            self.originalYawGoalTolerance = None
            self.discoveryYawParamDone = True
            self.discoveryYawParamSucceeded = False
            self.logErrorWithStatus(
                f"Exception while reading original yaw tolerance '{self.yawGoalToleranceParamName}': {e}. "
                f"The runtime update to 360 deg / 2pi rad will NOT be applied. "
                f"Discovery will continue anyway, but final yaw (at each single keypoint) may still be enforced/controlled by Nav2."
            )

    def discoverySetYawToleranceCallback(self, future):
        # In this callback, the SetParameters request is catched.
        try:
            response = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
            if len(response.results) != 1:
                self.discoveryYawParamDone = True
                self.discoveryYawParamSucceeded = False
                self.logErrorWithStatus(
                    f"Unexpected empty SetParameters response for parameter '{self.yawGoalToleranceParamName}'. "
                    f"Discovery will continue anyway, but final yaw (at each single keypoint) may still be enforced/controlled by Nav2."
                )
                return
            result = response.results[0]
            self.discoveryYawParamDone = True
            self.discoveryYawParamSucceeded = result.successful
            if self.discoveryYawParamSucceeded:
                self.logInfoWithStatus(f"{self.yawGoalToleranceParamName} set to 2pi rad. Original value was {self.originalYawGoalTolerance}.")
            else:
                self.logErrorWithStatus(
                    f"Failed to set '{self.yawGoalToleranceParamName}' to 360 deg / "
                    f"2pi rad. Reason from {params.controllerServerSetParametersServiceName}: {result.reason}. "
                    f"Discovery will continue anyway, but final yaw (at each single keypoint) may still be enforced/controlled by Nav2."
                )
        except Exception as e:
            self.discoveryYawParamDone = True
            self.discoveryYawParamSucceeded = False
            self.logErrorWithStatus(
                f"Exception while setting '{self.yawGoalToleranceParamName}' to 360 deg / 2pi rad: {e}."
                f"Discovery will continue anyway, but final yaw (at each single keypoint) may still be enforced/controlled by Nav2."
            )

    def handle_discoveryWaitYawToleranceSet(self):
        # In this state, the FSM simply waits for the completion of the runtime alteration of the YAW tolerance parameter of the Goal Checker.
        if not self.discoveryYawParamDone: return
        if not self.discoveryYawParamSucceeded:
            self.logWarnWithStatus("Yaw tolerance update for Goal Checker did not succeed. Continuing discovery anyway with the current Goal Checker configuration.")
        self.currentFSMstate = Task2FSMState.DISCOVERY_COMPUTE_PATH

    def handle_discoveryComputePath(self):
        # In this state, the FSM starts all the passages related to the discovery towards a new keypoint.
        # Indeed, this state will be reached multiple times during the discovery phase, processing all keypoints one by one.
        # Only after all keypoints have been processed, the FSM will pass to the final states.
        # Note that the discovery procedure is carried out exactly as it has been previosuly described.
        if self.discoveryCurrentGoalIndex >= len(self.discoveryOrderedGoals):
            # Discovery completed (all keypoints have been processed)!
            self.currentFSMstate = Task2FSMState.DISCOVERY_RESTORE_YAW_TOLERANCE
            return
        if not self.computePathActionClient.server_is_ready():
            self.logInfoWithStatus("Waiting for ComputePathToPose action server...")
            return
        # Building up the goal message for the ComputePathToPose Action Server
        targetNavGoal = self.discoveryOrderedGoals[self.discoveryCurrentGoalIndex]
        goalMsg = ComputePathToPose.Goal()
        goalMsg.goal = targetNavGoal.pose
        goalMsg.use_start = False # This FLAG indicated that the global path must be computed starting from the current robot pose
        # goalMsg.start = ... NOT used!
        self.discoveryPathDone = False
        self.discoveryPathSucceeded = False
        self.discoveryComputedPath = None
        self.logInfoWithStatus(f"Now requesting path for goal/keypoint with index {self.discoveryCurrentGoalIndex}...")
        future = self.computePathActionClient.send_goal_async(goalMsg)
        future.add_done_callback(self.discoveryComputePathGoalResponseCallback)
        self.currentFSMstate = Task2FSMState.DISCOVERY_WAIT_PATH

    def discoveryComputePathGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if not goalHandle.accepted:
            self.logErrorWithStatus("ComputePathToPose goal unexpectedly rejected.")
            self.discoveryPathDone = True
            self.discoveryPathSucceeded = False
            return
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.discoveryComputePathResultCallback)

    def discoveryComputePathResultCallback(self, future):
        wrappedResult = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        self.discoveryPathDone = True
        self.discoveryPathSucceeded = (wrappedResult.status == GoalStatus.STATUS_SUCCEEDED)
        if self.discoveryPathSucceeded:
            self.discoveryComputedPath = wrappedResult.result.path
            self.logInfoWithStatus("ComputePathToPose received successfully!")
        else:
            self.discoveryComputedPath = None
            self.logErrorWithStatus(f"ComputePathToPose failed with status {wrappedResult.status}.")

    def handle_discoveryWaitPath(self):
        # In this state, the FSM simply waits for the completion of the ComputePathToPose Action Server request.
        # Note that if the request fails, then it is assumed that no possibile path to be travelled exists and the current keypoint is skipped in the discovery process.
        # Otherwise, if the request succeeds, then the FSM passes to the next state to re-orient the robot towards the computed path heading.
        if not self.discoveryPathDone: return
        if not self.discoveryPathSucceeded or self.discoveryComputedPath is None:
            self.logWarnWithStatus(f"The ComputePathToPose request failed. The current goal ({self.discoveryCurrentGoalIndex}) is therefore skipped.")
            self.discoveryCurrentGoalIndex += 1
            self.currentFSMstate = Task2FSMState.DISCOVERY_COMPUTE_PATH
            return
        self.currentFSMstate = Task2FSMState.DISCOVERY_SPIN_TO_PATH_HEADING

    def handle_discoverySpinToPathHeading(self):
        # In this state, the FSM exploits the Spin Action Server to re-orient the robot towards the heading of the computed path.
        if not self.spinActionClient.server_is_ready():
            self.logInfoWithStatus("Waiting for Spin Action Server...")
            return
        if self.lastAMCLMsg is None:
            self.logWarnWithStatus("Missing last AMCL pose, waiting for it to be available...")
            return
        # Extracting the heading of the computed path (with the previously defined method)
        targetHeading = self.extractInitialPathHeading(self.discoveryComputedPath)
        if targetHeading is None:
            # In case is NOT possible to retrieve the heading of the computed path, then the FSM simply skips the spin pahse (and immediatly navigates to the next keypoint).
            self.logWarnWithStatus("It was not possibile to extract the path heading. Skipping the Spin action and navigating towards the next keypoint.")
            self.currentFSMstate = Task2FSMState.DISCOVERY_NAVIGATE_TO_GOAL
            return
        # Retreiving the current yaw of the robot and computing the longest spin to reach the target heading (with the previously defined methods)
        currentYaw = self.getCurrentYawFromAMCLmessages()
        longSpin = self.computeLongSpin(currentYaw, targetHeading)
        # Building up the goal message for the Spin Action Server
        goalMsg = Spin.Goal()
        goalMsg.target_yaw = float(longSpin)
        goalMsg.time_allowance = Duration(sec = 30)
        self.discoverySpinDone = False
        self.discoverySpinSucceeded = False
        self.logInfoWithStatus(f"Requiring a Spin to the Nav2 Stack: currentYaw = {currentYaw:.3f}, targetHeading={targetHeading:.3f}, longSpin={longSpin:.3f}")
        future = self.spinActionClient.send_goal_async(goalMsg)
        future.add_done_callback(self.discoverySpinGoalResponseCallback)
        self.currentFSMstate = Task2FSMState.DISCOVERY_WAIT_SPIN

    def discoverySpinGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if not goalHandle.accepted:
            self.logErrorWithStatus("Spin goal unexpectedly rejected.")
            self.discoverySpinDone = True
            self.discoverySpinSucceeded = False
            return
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.discoverySpinResultCallback)

    def discoverySpinResultCallback(self, future):
        wrappedResult = future.result()
        self.discoverySpinDone = True
        self.discoverySpinSucceeded = (wrappedResult.status == GoalStatus.STATUS_SUCCEEDED)
        self.logInfoWithStatus(f"Spin completed with status {wrappedResult.status}.")

    def handle_discoveryWaitSpin(self):
        # In this state, the FSM simply waits for the completion of the Spin Action Server request,
        # then it passes to the next state to navigate towards the current keypoint.
        if not self.discoverySpinDone: return
        if not self.discoverySpinSucceeded:
            self.logWarnWithStatus("The Spin Action has been sent BUT has failed; navigating anyway to the next keypoint.")
        self.currentFSMstate = Task2FSMState.DISCOVERY_NAVIGATE_TO_GOAL

    def handle_discoveryNavigateToGoal(self):
        # In this state, the FSM exploits the NavigateToPose Action Server to navigate towards the current/new-to-be-reached keypoint.
        if not self.navigateToPoseActionClient.server_is_ready():
            self.logInfoWithStatus("Waiting for NavigateToPose action server...")
            return
        goalMsg = self.discoveryOrderedGoals[self.discoveryCurrentGoalIndex]
        goalMsg.pose.header.stamp = self.get_clock().now().to_msg()
        self.discoveryNavDone = False
        self.discoveryNavSucceeded = False
        self.logInfoWithStatus(f"Navigating to goal/keypoint with index {self.discoveryCurrentGoalIndex}...")
        future = self.navigateToPoseActionClient.send_goal_async(goalMsg)
        future.add_done_callback(self.discoveryNavigateGoalResponseCallback)
        self.currentFSMstate = Task2FSMState.DISCOVERY_WAIT_NAVIGATION

    def discoveryNavigateGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if not goalHandle.accepted:
            self.logErrorWithStatus("NavigateToPose goal unexpectedly rejected.")
            self.discoveryNavDone = True
            self.discoveryNavSucceeded = False
            return
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.discoveryNavigateResultCallback)

    def discoveryNavigateResultCallback(self, future):
        wrappedResult = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        self.discoveryNavDone = True
        self.discoveryNavSucceeded = (wrappedResult.status == GoalStatus.STATUS_SUCCEEDED)
        self.logInfoWithStatus(f"Navigation completed with status {wrappedResult.status}.")

    def handle_discoveryWaitNavigation(self):
        # In this state, the FSM simply waits for the completion of the NavigateToPose Action Server request (both with a success or a failure),
        # then it passes to the next state to compute the path towards the next keypoint.
        # In case ALL keypoints have been processed, then the FSM passes to the final states of the discovery procedure,
        # that is the restoration of the original YAW tolerance parameter of the Goal Checker.
        if not self.discoveryNavDone: return
        if self.discoveryNavSucceeded: self.logInfoWithStatus(f"Goal/keypoint with index {self.discoveryCurrentGoalIndex} successfully reached.")
        else: self.logWarnWithStatus(f"Navigation to goal/keypoint with index {self.discoveryCurrentGoalIndex} unexpectedly failed. Passing to the next keypoint.")
        self.discoveryCurrentGoalIndex += 1
        if self.discoveryCurrentGoalIndex >= len(self.discoveryOrderedGoals):
            self.logInfoWithStatus("All discovery keypoints have been processed. Now restoring the original yaw_goal_tolerance parameter of the Goal Checker...")
            self.currentFSMstate = Task2FSMState.DISCOVERY_RESTORE_YAW_TOLERANCE
        else: self.currentFSMstate = Task2FSMState.DISCOVERY_COMPUTE_PATH

    def handle_discoveryRestoreYawTolerance(self):
        # In this state, the FSM restores the original value of the YAW tolerance parameter of the Goal Checker (that was previously altered to 360 deg / 2pi rad).
        if self.originalYawGoalTolerance is None:
            self.logWarnWithStatus(
                f"Original value of '{self.yawGoalToleranceParamName}' is unknown, "
                f"so no restore can be applied. Note that this indicates that the initial read or the initial update to 360 deg failed."
            )
            self.currentFSMstate = Task2FSMState.DISCOVERY_FINISHED
            return
        if not self.controllerServerSetParametersClient.service_is_ready():
            self.logInfoWithStatus(f"Waiting for {params.controllerServerSetParametersServiceName} service...")
            return
        self.discoveryRestoreYawDone = False
        self.discoveryRestoreYawSucceeded = False
        # Building up a SetParameters request for restoring the original value of the YAW tolerance parameter of the Goal Checker
        req = SetParameters.Request()
        param = Parameter(
            self.yawGoalToleranceParamName,
            Parameter.Type.DOUBLE,
            float(self.originalYawGoalTolerance)
        )
        req.parameters = [param.to_parameter_msg()]
        future = self.controllerServerSetParametersClient.call_async(req)
        future.add_done_callback(self.discoveryRestoreYawToleranceCallback)
        self.currentFSMstate = Task2FSMState.DISCOVERY_WAIT_YAW_TOLERANCE_RESTORED

    def discoveryRestoreYawToleranceCallback(self, future):
        # In this callback, the SetParameters request for restoring the original value of the YAW tolerance parameter of the Goal Checker is catched
        try:
            response = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
            if len(response.results) != 1:
                self.discoveryRestoreYawDone = True
                self.discoveryRestoreYawSucceeded = False
                self.logErrorWithStatus(
                    f"Unexpected empty SetParameters response: the restoration of '{self.yawGoalToleranceParamName}' to "
                    f"its original value {self.originalYawGoalTolerance} may have failed.")
                return
            result = response.results[0]
            self.discoveryRestoreYawDone = True
            self.discoveryRestoreYawSucceeded = result.successful
            if result.successful: self.logInfoWithStatus(f"Restored {self.yawGoalToleranceParamName} to {self.originalYawGoalTolerance}.")
            else:
                self.logErrorWithStatus(
                    f"Failed to restore '{self.yawGoalToleranceParamName}' to original value {self.originalYawGoalTolerance}."
                    f"Reason from {params.controllerServerSetParametersServiceName}: {result.reason}."
                )
        except Exception as e:
            self.discoveryRestoreYawDone = True
            self.discoveryRestoreYawSucceeded = False
            self.logErrorWithStatus(f"Exception while restoring '{self.yawGoalToleranceParamName}' to original value {self.originalYawGoalTolerance}: {e}.")

    def handle_discoveryWaitYawToleranceRestored(self):
        # In this penultimate state, the FSM simply waits for the completion of the SetParameters request
        # (for restoring the original value of the YAW tolerance parameter of the Goal Checker),
        # then it passes to the final state of the discovery procedure, that is the saving of all the discovery results!
        if not self.discoveryRestoreYawDone: return
        if not self.discoveryRestoreYawSucceeded: self.logWarnWithStatus("Attention: restore of original yaw_goal_tolerance failed.")
        self.currentFSMstate = Task2FSMState.DISCOVERY_FINISHED

    def handle_discoveryFinished(self):
        # In this final state, the FSM simply saves all the results of the discovery procedure (if not already done) and then it sets a flag to finally shutdown the node.
        self.logInfoWithStatus("Discovery navigation completed.")
        if not self.arucoLocationsSaved: self.saveArucoLocations()
        self.shouldShutdown = True

def main(args = None):
    node = None
    try:
        rclpy.init(args = args)
        node = Task2FSMNode()
        while rclpy.ok() and not node.shouldShutdown: rclpy.spin_once(node, timeout_sec = 0.1) # spin_once() spins (IF present, otherwise it waits once the indicated time) the single first available callback, then immediatly returns 
    except KeyboardInterrupt:
        if node is not None: node.get_logger().info(f"KeyboardInterrupt received, shutting down {params.task2FSMNodeName}...")
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__": main()