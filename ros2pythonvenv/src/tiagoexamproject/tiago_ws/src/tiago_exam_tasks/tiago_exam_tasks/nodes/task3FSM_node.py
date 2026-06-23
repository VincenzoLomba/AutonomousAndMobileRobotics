
# This Python files defines a ROS2 Humble Node which implements the FSM that implements the logic that executes the Task3

import math
import random
from enum import Enum
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from . import tiagoExamParameters as params
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from tiago_exam_interfaces.action import TiagoArm, TiagoGripper
from dataclasses import dataclass, field
from typing import Optional
from rclpy.time import Time
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, asdict
import discoveryLoader
import numpy as np
from scipy.spatial.transform import Rotation as SciPyRotation
from geometry_msgs.msg import Pose
import copy

@dataclass
class ArucoSample:
    # This simple class represents the single sample collected via an Aruco Marker detection
    stamp_sec: int # Seconds part of the ROS timestamp associated with the transform message.
    stamp_nanosec: int # Nanoseconds part of the ROS timestamp associated with the transform message.
    frame_id: str # Reference frame in which the marker pose is expressed, typically "map".
    child_frame_id: str # TF child frame associated with the detected ArUco marker.
    position: List[float] # Marker position [x, y, z] expressed in frame_id, in meters.
    quaternion: List[float] # Normalized marker orientation [x, y, z, w] expressed in frame_id.

@dataclass
class CubeData:
    # This class will be used to organize and collect all the relevant data for the currently selected cube.
    # Note that the currently selected cube is the one cube with which the Tiago robot is actually interacting!
    markerInfo: params.MarkerInfo # This is the set of Aruco Marker informations associated to the cube
    arucoSamples: list[ArucoSample] = field(default_factory=list) # This list will be used to store all the collected ArUco marker samples for the currently selected cube
    detectionWaitStartTime: Optional[Time] = None # During the Task3 execution, at a certain moment the Tiago robot will wait a certain time for collecting Aruco samples for the currently selected cube; this variable will store the time at which this waiting phase starts
    finalPoseMap: Optional[params.ArucoPoseEstimate] = None # Final estimated pose of the currently selected cube (of its marker) expressed in the "map" reference frame (obtained by aggregating all the collected samples for the currently selected cube)
    finalPoseBaseLink: Optional[params.ArucoPoseEstimate] = None # Same as the previous variable, BUT expressed in the "base_link" reference frame
    approachPose: Optional[Pose] = None # Apporach pose for the cube (espressed in the "base_link" reference frame)
    # Important: in the final version of the code, the grasping pose is reached simply by a vertical translation of the Tiago torso FROM the approach pose
    # graspingPosition: Optional[list[float]] = None # Position part of the grasping pose for the cube
    # graspingQuaternion: Optional[list[float]] = None # Orientation part of the grasping pose for the cube
    @property
    def markerID(self) -> int: return self.markerInfo.markerID
    @property
    def markerNickname(self) -> str: return self.markerInfo.markerNickname

class Task3FSMState(Enum):
    # Autonomous localization (copied from Task2 FSM)
    TUCK_ARM = 1
    WAIT_ARM_TUCKED = 2
    TRIGGER_LOCALIZATION_RESTART = 3
    WAIT_LOCALIZATION_RESTARTED = 4
    EVALUATING_LOCALIZATION = 5
    LOCALIZING = 6
    # Cubes transportation between pick and place locations
    INIT_CUBES_TRANSPORTATION = 7
    NAVIGATE_TO_PICK_PLATFORM = 10
    CUBE_DETECTION = 12
    MOVE_ARM_TO_PICKING_CONFIGURATION = 14



    WAIT_HEAD_FOR_TASK3 = 8
    LOAD_TASK2_DISCOVERY_RESULTS = 9
    
    WAIT_PICK_PLATFORM_NAVIGATION = 11
    
    BUILD_CURRENT_CUBE_APPROACH_POSE = 13
    
    WAIT_ARM_TO_PICKING_CONFIGURATION = 15
    MOVE_ARM_TO_CURRENT_CUBE_APPROACH = 16
    WAIT_ARM_TO_CURRENT_CUBE_APPROACH = 17
    OPEN_GRIPPER = 18
    WAIT_GRIPPER_OPENED = 19
    BUILD_CURRENT_CUBE_GRASPING_POSE = 20
    MOVE_ARM_TO_CURRENT_CUBE_GRASPING = 21
    WAIT_ARM_TO_CURRENT_CUBE_GRASPING = 22
    CLOSE_GRIPPER = 23
    WAIT_GRIPPER_CLOSED = 24
    MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH = 25
    WAIT_ARM_BACK_TO_CURRENT_CUBE_APPROACH = 26
    MOVE_ARM_TO_TRANSPORT_CONFIGURATION = 27
    WAIT_ARM_TO_TRANSPORT_CONFIGURATION = 28
    NAVIGATE_TO_PLACE_PLATFORM = 29
    WAIT_PLACE_PLATFORM_NAVIGATION = 30
    MOVE_ARM_TO_PLACE_APPROACH = 31
    WAIT_ARM_TO_PLACE_APPROACH = 32
    MOVE_TORSO_TO_PLACE = 33
    WAIT_TORSO_TO_PLACE = 34
    OPEN_GRIPPER_FOR_PLACE = 35
    WAIT_GRIPPER_OPENED_FOR_PLACE = 36
    MOVE_TORSO_BACK_FROM_PLACE = 37
    WAIT_TORSO_BACK_FROM_PLACE = 38
    CLOSE_GRIPPER_AFTER_PLACE = 39
    WAIT_GRIPPER_CLOSED_AFTER_PLACE = 40
    MOVE_ARM_TO_HOME_AFTER_PLACE = 41
    WAIT_ARM_TO_HOME_AFTER_PLACE = 42
    ADVANCE_TO_NEXT_CUBE = 43
    RETURN_TO_INITIAL_POSE = 44
    WAIT_RETURN_TO_INITIAL_POSE = 45
    FINISH = 46

class Task3FSMNode(Node):

    def __init__(self):

        super().__init__(params.task3FSMNodeName)
        self.get_logger().info(f"Starting {params.task3FSMNodeName} initialization...")

        # +---------------------------------------------------------------------------------------------------------------------------------------------+
        # | The following portion of code is exactly the same as the one used in the Task2 FSM Node for (1) arm tucking and (2) autonomous localization |
        # +---------------------------------------------------------------------------------------------------------------------------------------------+

        self.lastAMCLMsg = None
        self.latestJointState = None
        self.collectArucoSamples = False
        self.pickArucoSamples = []
        self.placeArucoSamples = []
        self.arucoLocationsSaved = False
        self.armTuckGoalDone = False
        self.armTuckGoalSucceeded = False
        self.globalLocalizationResetDone = False
        self.globalLocalizationResetSucceeded = False
        self.localizationMetricsHistory = []
        self.localizationMetricsWindowSize = params.localizationMetricsWindowSize
        self.localizationMetricsJumpUpResetPercent = params.localizationMetricsJumpUpResetPercent
        self.localizationMetricsMinImprovementPercent = params.localizationMetricsMinImprovementPercent
        self.collisionMonitorRequestSent = False
        self.collisionMonitorResponseReceived = False
        self.collisionMonitorEnabled = False
        self.localizationPhase = None
        self.localizationGoodMetricConsecutiveCountRequired = 2 
        self.localizationGoodMetricConsecutiveCount = params.localizationGoodMetricConsecutiveCountRequired
        self.robotXY = (0.0, 0.0)
        self.localizationSpinDone = False
        self.localizationSpinSucceeded = False
        self.localizationDriveOnHeadingDone = False
        self.localizationDriveOnHeadingSucceeded = False
        self.RANDOMWALKlocalization = False
        self.headTiltTolerance = params.headTiltTolerance

        self.shouldShutdown = False

        # +------------------------------------------------------------------------------------------------------------------------+
        # |                                          End of the reused part fo the code                                            |
        # +------------------------------------------------------------------------------------------------------------------------+
        
        # This variable will be used to enable/disable the collection of the ArUco marker samples for the currently selected cube
        self.collectCurrentCubeSamplesENABLER = False 
        self.currentCubeIndex = 0 # This is the index of the currently selected cube, AKA the cube that the Tiago robot is actually interacting with
        self.cubesData = [] # This list will store the data for all the cubes that the Tiago robot will have to pick and place (and will be indexed via self.currentCubeIndex)
        # Initialize the internal state (and related quantities)
        if not params.cubes or len(params.cubes) == 0:
            self.get_logger().error("The provided cubes list (to be pick and placed) seems to be empty! Provide a valid list!")
            self.currentFSMstate = Task3FSMState.FINISH
        else:
            self.cubesData = [CubeData(markerInfo = cube) for cube in params.cubes]
            self.currentFSMstate = Task3FSMState.TUCK_ARM

        self.pickLocation: params.ArucoPoseEstimate = None # In this variable it will be sotred the loaded estimated pose of the pick location
        self.placeLocation: params.ArucoPoseEstimate = None # In this variable it will be sotred the loaded estimated pose of the place location

        self.tiltHeadSent = False
        
        self.pickPlatformNavigationSent = False
        self.pickPlatformNavigationDone = False
        self.pickPlatformNavigationSucceeded = False

        self.placePlatformNavigationSent = False
        self.placePlatformNavigationDone = False
        self.placePlatformNavigationSucceeded = False

        self.armPickingGoalDone = False
        self.armPickingGoalSucceeded = False

        self.armApproachGoalDone = False
        self.armApproachGoalSucceeded = False

        self.gripperOpenGoalDone = False
        self.gripperOpenGoalSucceeded = False

        self.gripperCloseGoalDone = False
        self.gripperCloseGoalSucceeded = False

        self.armGraspingGoalDone = False
        self.armGraspingGoalSucceeded = False

        self.armReturnApproachGoalDone = False
        self.armReturnApproachGoalSucceeded = False

        self.armTransportGoalDone = False
        self.armTransportGoalSucceeded = False

        self.armPlaceApproachGoalDone = False
        self.armPlaceApproachGoalSucceeded = False

        self.armHomeAfterPlaceGoalDone = False
        self.armHomeAfterPlaceGoalSucceeded = False

        self.nextStateAfterTransportConfiguration = None

        # During the transportation, the torso of the Tiago robot will be moved. The following variables will be involved in that motion.
        self.torsoMotionTarget = None # Last set target position for the torso motion.
        self.torsoMotionStartTime = None # Instant at which the torso motion was commanded (used to check for timeout).
        self.torsoMotionTimeout = 8.0 # Maximum time allowed for the single torso motion to complete (in seconds).
        self.torsoMotionTolerance = 0.005 # Tolerance for considering the single torso motion as completed (in meters).

        self.initialRobotPose = None # In this variable, the whole POSE of the robot BEFORE starting the transportation will be stored

        

        self.torsoApproachJointPosition = None
        self.torsoMinSafePosition = 0.005


        # Initialize the TF broadcaster to be able to publish STATIC TFs
        self.tfBroadcaster = StaticTransformBroadcaster(self)
        # Initialize the TF buffer and listener to be able to listen to the TFs published by other nodes (e.g. the ArUco Marker detection nodes)
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        # Initialize the Action Client for the TiagoGripper Action Server (exposed by the TiagoGripper Node) to be able to send goal for the gripper motion
        self.tiagoGripperActionClient = ActionClient(self, TiagoGripper, params.tiagoGripperActionName)
        # Subscribe to the Aruco Marker detection topics for all the required cubes
        self.cubesArucoSubscriptionsDictionary = {}
        for cube in params.cubes:
            self.cubesArucoSubscriptionsDictionary[cube.markerID] = self.create_subscription(
                TransformStamped, cube.getTopicTF(), 
                lambda msg, markerID = cube.markerID: self.storeCubeMarkerSample(msg, markerID), 10,
            )
        # Initialize the Publisher for commanding the torso position AKA publisher to the "/torso_controller/joint_trajectory" topic
        self.torsoCommandPublisher = self.create_publisher(JointTrajectory, params.torsoCommandTopic, 10)
            
        # +-----------------------------------------------------------------------------------------+
        # | The following portion of code is exactly the same as the one used in the Task2 FSM Node |
        # +-----------------------------------------------------------------------------------------+

        # Initialize the Action Client for the TiagoArm Action Server (exposed by the TiagoArm Node) to be able to send goal for the arm motion
        self.tiagoArmActionClient = ActionClient(self, TiagoArm, params.tiagoArmActionName) # Action Client
        # Subscribe to the "amcl_pose" topic to get the robot's estimated position (AMCL = Adaptive Monte Carlo Localization) (note that this is a PLANAR localizator)
        self.AMCLPoseSubscription = self.create_subscription(PoseWithCovarianceStamped, params.amclPoseTopic, self.amclCallback, 10)
        # Subscribe to the "joint_states" topic to get the current joint states of the robot
        self.jointStateSubscription = self.create_subscription(JointState, params.jointStateTopic, self.jointStateCallback, 10)
        # Subscribe to the Aruco Marker detection topics for both pick and place locations to get the detected marker poses
        # self.pickArucoSubscription = self.create_subscription(TransformStamped, '/aruco_pick/single/transform', self.pickArucoCallback, 10)
        # self.placeArucoSubscription = self.create_subscription(TransformStamped, '/aruco_place/single/transform', self.placeArucoCallback, 10)
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
        # Initialize the Action Client for the ComputePathToPose Action Server provided by the Nav2 Stack
        # self.computePathActionClient = ActionClient(self, ComputePathToPose, params.computePathToPoseActionName)
        # Initialize the Action Client for the NavigateToPose Action Server provided by the Nav2 Stack
        self.navigateToPoseActionClient = ActionClient(self, NavigateToPose, params.navigateToPoseActionName)

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
        self.get_logger().info(f"Task3 FSM node initialized. Initial state: {self.currentFSMstate.name}.")

        # +-----------------------------------------------------------------------------------------+
        # |             End of the reused part fo the code (and end of the constructor)             |
        # +-----------------------------------------------------------------------------------------+
    
    def logInfoWithStatus(self, message): self.get_logger().info(f"[{self.currentFSMstate.name}] {message}")
    def logWarnWithStatus(self, message): self.get_logger().warn(f"[{self.currentFSMstate.name}] {message}")
    def logErrorWithStatus(self, message): self.get_logger().error(f"[{self.currentFSMstate.name}] {message}")

    def amclCallback(self, msg): self.lastAMCLMsg = msg
    def jointStateCallback(self, msg): self.latestJointState = msg

    def storeCubeMarkerSample(self, msg: TransformStamped, markerID: int):
        # This method takes care of the collected Aruco Marker samples (for the currently selected cube) and stores them in the corresponding list
        currentCubeData = self.cubesData[self.currentCubeIndex]
        if not self.collectCurrentCubeSamplesENABLER or markerID != currentCubeData.markerID: return
        t = msg.transform.translation
        q = msg.transform.rotation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm <= 1e-9:
            self.logWarnWithStatus(f"Ignoring {currentCubeData.markerNickname} marker {currentCubeData.markerID} sample with invalid [almost] zero-norm quaternion.")
            return
        self.cubesData[self.currentCubeIndex].arucoSamples.append(ArucoSample(
            stamp_sec = int(msg.header.stamp.sec), # Seconds part of the ROS timestamp associated with the transform message.
            stamp_nanosec = int(msg.header.stamp.nanosec), # Nanoseconds part of the ROS timestamp associated with the transform message.
            frame_id = str(msg.header.frame_id), # Reference frame in which the marker pose is expressed, typically "map".
            child_frame_id = str(msg.child_frame_id), # TF child frame associated with the detected ArUco marker.
            position = [float(t.x), float(t.y), float(t.z)], # Marker position [x, y, z] expressed in frame_id, in meters.
            quaternion = [float(q.x / norm), float(q.y / norm), float(q.z / norm), float(q.w / norm)], # Normalized marker orientation [x, y, z, w] expressed in frame_id.
        ))

    def stepUpFSM(self):
        # Autonomous localization (copied from Task2 FSM)
        if self.currentFSMstate == Task3FSMState.TUCK_ARM: self.handle_tuckArm()
        elif self.currentFSMstate == Task3FSMState.WAIT_ARM_TUCKED: self.handle_waitArmTucked()
        elif self.currentFSMstate == Task3FSMState.TRIGGER_LOCALIZATION_RESTART: self.handle_triggerLocalization()
        elif self.currentFSMstate == Task3FSMState.WAIT_LOCALIZATION_RESTARTED: self.handle_waitLocalizationTriggered()
        elif self.currentFSMstate == Task3FSMState.EVALUATING_LOCALIZATION: self.handle_evaluatingLocalization()
        elif self.currentFSMstate == Task3FSMState.LOCALIZING: self.handle_localizing()
        # Cubes transportation between pick and place locations
        # Note that in the following FSM states I have used a DIFFERENT standard approach w.r.t previous ones:
        # I do NOT split anymore the single action in a couple of execute/wait states, but I handle that couple in an unique state.
        # In order to do that, I have added an additional FLAG for each state/action (the "sent" one)
        elif self.currentFSMstate == Task3FSMState.INIT_CUBES_TRANSPORTATION: self.handle_initCubesTransportation()
        elif self.currentFSMstate == Task3FSMState.NAVIGATE_TO_PICK_PLATFORM: self.handle_navigateToPickPlatform()
        elif self.currentFSMstate == Task3FSMState.CUBE_DETECTION: self.handle_cubeDetection()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_PICKING_CONFIGURATION: self.handle_moveArmToPickingConfiguration()



        elif self.currentFSMstate == Task3FSMState.WAIT_ARM_TO_PICKING_CONFIGURATION:
            self.handle_waitArmToPickingConfiguration()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_CURRENT_CUBE_APPROACH:
            self.handle_moveArmToCurrentCubeApproach()
        elif self.currentFSMstate == Task3FSMState.WAIT_ARM_TO_CURRENT_CUBE_APPROACH:
            self.handle_waitArmToCurrentCubeApproach()
        elif self.currentFSMstate == Task3FSMState.OPEN_GRIPPER:
            self.handle_openGripper()
        elif self.currentFSMstate == Task3FSMState.WAIT_GRIPPER_OPENED:
            self.handle_waitGripperOpened()
        elif self.currentFSMstate == Task3FSMState.BUILD_CURRENT_CUBE_GRASPING_POSE:
            self.handle_buildCurrentCubeGraspingPose()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_CURRENT_CUBE_GRASPING:
            self.handle_moveArmToCurrentCubeGrasping()
        elif self.currentFSMstate == Task3FSMState.WAIT_ARM_TO_CURRENT_CUBE_GRASPING:
            self.handle_waitArmToCurrentCubeGrasping()
        elif self.currentFSMstate == Task3FSMState.CLOSE_GRIPPER:
            self.handle_closeGripper()
        elif self.currentFSMstate == Task3FSMState.WAIT_GRIPPER_CLOSED:
            self.handle_waitGripperClosed()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH:
            self.handle_moveArmBackToCurrentCubeApproach()
        elif self.currentFSMstate == Task3FSMState.WAIT_ARM_BACK_TO_CURRENT_CUBE_APPROACH:
            self.handle_waitArmBackToCurrentCubeApproach()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_TRANSPORT_CONFIGURATION:
            self.handle_moveArmToTransportConfiguration()
        elif self.currentFSMstate == Task3FSMState.WAIT_ARM_TO_TRANSPORT_CONFIGURATION:
            self.handle_waitArmToTransportConfiguration()
        elif self.currentFSMstate == Task3FSMState.NAVIGATE_TO_PLACE_PLATFORM:
            self.handle_navigateToPlacePlatform()
        elif self.currentFSMstate == Task3FSMState.WAIT_PLACE_PLATFORM_NAVIGATION:
            self.handle_waitPlacePlatformNavigation()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_PLACE_APPROACH:
            self.handle_moveArmToPlaceApproach()
        elif self.currentFSMstate == Task3FSMState.WAIT_ARM_TO_PLACE_APPROACH:
            self.handle_waitArmToPlaceApproach()
        elif self.currentFSMstate == Task3FSMState.MOVE_TORSO_TO_PLACE:
            self.handle_moveTorsoToPlace()
        elif self.currentFSMstate == Task3FSMState.WAIT_TORSO_TO_PLACE:
            self.handle_waitTorsoToPlace()
        elif self.currentFSMstate == Task3FSMState.OPEN_GRIPPER_FOR_PLACE:
            self.handle_openGripperForPlace()
        elif self.currentFSMstate == Task3FSMState.WAIT_GRIPPER_OPENED_FOR_PLACE:
            self.handle_waitGripperOpenedForPlace()
        elif self.currentFSMstate == Task3FSMState.MOVE_TORSO_BACK_FROM_PLACE:
            self.handle_moveTorsoBackFromPlace()
        elif self.currentFSMstate == Task3FSMState.WAIT_TORSO_BACK_FROM_PLACE:
            self.handle_waitTorsoBackFromPlace()
        elif self.currentFSMstate == Task3FSMState.CLOSE_GRIPPER_AFTER_PLACE:
            self.handle_closeGripperAfterPlace()
        elif self.currentFSMstate == Task3FSMState.WAIT_GRIPPER_CLOSED_AFTER_PLACE:
            self.handle_waitGripperClosedAfterPlace()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_HOME_AFTER_PLACE:
            self.handle_moveArmToHomeAfterPlace()
        elif self.currentFSMstate == Task3FSMState.WAIT_ARM_TO_HOME_AFTER_PLACE:
            self.handle_waitArmToHomeAfterPlace()
        elif self.currentFSMstate == Task3FSMState.ADVANCE_TO_NEXT_CUBE:
            self.handle_advanceToNextCube()
        elif self.currentFSMstate == Task3FSMState.RETURN_TO_INITIAL_POSE:
            self.handle_returnToInitialPose()
        elif self.currentFSMstate == Task3FSMState.WAIT_RETURN_TO_INITIAL_POSE:
            self.handle_waitReturnToInitialPose()
        elif self.currentFSMstate == Task3FSMState.FINISH:
            self.handle_finish()
        else:
            self.get_logger().error(f"Encountered an unknown FSM state: {self.currentFSMstate}")

    # +--------------------------------------------------------------------------------------------------------------------------+
    # | The following portion of code related to the tuck of the arm is exactly the same as the one used in the Task1/2 FSM Node |
    # +--------------------------------------------------------------------------------------------------------------------------+

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
        self.currentFSMstate = Task3FSMState.WAIT_ARM_TUCKED

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
                self.currentFSMstate = Task3FSMState.TRIGGER_LOCALIZATION_RESTART
            else:
                self.logWarnWithStatus("Arm goal failed or was rejected by TiagoArm Action Server. Retrying...")
                self.currentFSMstate = Task3FSMState.TUCK_ARM

    # +--------------------------------------------------------------------------------------------------------------------------+
    # |                             End of the part fo the code related to the tuck of the arm                                   |
    # +--------------------------------------------------------------------------------------------------------------------------+

    # +----------------------------------------------------------------------------------------------------------------------------+
    # | The following portion of code related to Autonomous Localization is exactly the same as the one used in the Task2 FSM Node |
    # +----------------------------------------------------------------------------------------------------------------------------+
    # Note: I also left all the comments (I was lazy and I didn't want to remove them)

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
        self.currentFSMstate = Task3FSMState.WAIT_LOCALIZATION_RESTARTED

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
                self.currentFSMstate = Task3FSMState.EVALUATING_LOCALIZATION
            else:
                self.logErrorWithStatus("n |>\n |> Global localization reset failed, now retrying...\n |>")
                self.currentFSMstate = Task3FSMState.TRIGGER_LOCALIZATION_RESTART
    
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
            self.get_logger().info("\n |>\n |> Collision monitor callback completed\n |>")
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
                    self.currentFSMstate = Task3FSMState.SET_HEAD_FOR_DISCOVERY
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

                if not self.collisionMonitorRequestSent and not self.collisionMonitorResponseReceived: self.currentFSMstate = Task3FSMState.LOCALIZING
    
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
        self.get_logger().info("[LOCALIZING] Spin goal accepted by Nav2.")
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
                self.currentFSMstate = Task3FSMState.EVALUATING_LOCALIZATION
            else:
                self.logErrorWithStatus("Spin movement failed... anyway, now going back to evaluate localization quality and deciding the next step...")
                self.currentFSMstate = Task3FSMState.EVALUATING_LOCALIZATION
        if self.localizationDriveOnHeadingDone:
            if not self.RANDOMWALKlocalization:
                # Note this important behaviour: in case of NOT relying on a random-walk-like approach (as it IS in the final version of the code),
                # then after each single DriveOnHeading Action, the FSM state is set to "TRIGGER_LOCALIZATION_RESTART".
                # This is exactly as expected and commented out before: after the position change, the autonomous localization procedure is restarted from scratch.
                self.currentFSMstate = Task3FSMState.TRIGGER_LOCALIZATION_RESTART
            elif self.localizationDriveOnHeadingSucceeded:
                self.logInfoWithStatus("DriveOnHeading movement completed successfully, now going back to evaluate localization quality...")
                self.currentFSMstate = Task3FSMState.EVALUATING_LOCALIZATION
            else:
                self.logErrorWithStatus("DriveOnHeading movement failed... anyway, now going back to evaluate localization quality and deciding the next step...")
                self.currentFSMstate = Task3FSMState.EVALUATING_LOCALIZATION

    def getJointPosition(self, jointName):
        # A very simple method that exploit the last available JointState message to extract the current position of a given joint
        # If no info is available yet, or if it's not retrievable, then None is returned
        if self.latestJointState is None: return None
        try:
            idx = self.latestJointState.name.index(jointName)
            return float(self.latestJointState.position[idx])
        except (ValueError, IndexError): return None
    
    # +----------------------------------------------------------------------------------------------------------------------------+
    # |                             End of the part fo the code related to Autonomous Localization                                 |
    # +----------------------------------------------------------------------------------------------------------------------------+

    # +------------------------------------------------------------------------------------------------------------------------+
    # |                      Transportation of all cubes from the Pick Location to the Place Location                          |
    # +------------------------------------------------------------------------------------------------------------------------+

    def publishTorsoliftCommand(self, targetPosition: float, label: str):
        # This method publishes a command to the torso lift joint to move it to the specified target position
        msg = JointTrajectory()
        msg.joint_names = [params.torsoLiftJointName]
        point = JointTrajectoryPoint()
        point.positions = [float(targetPosition)]
        point.time_from_start = Duration(sec = 2) # Imposed time duration for the head movement
        msg.points.append(point)
        self.torsoCommandPublisher.publish(msg)
        self.torsoMotionTarget = float(targetPosition)
        self.torsoMotionStartTime = self.get_clock().now()
        self.logInfoWithStatus(f"Published {params.torsoLiftJointName} command, target to {float(targetPosition):.4f}.")

    def isTorsoMotionCompleted(self):
        # This method can be used to check if the last published command for the torso motion has been already completed.
        # Two boolean values are returned: done (indicating if the motion has to be considered as completed or not) and success (True IFF the motion is successful).
        # Note that in case done==False, then surely also success==False
        if self.torsoMotionTarget is None or self.torsoMotionStartTime is None:
            self.logErrorWithStatus("A request for check the complention of the torso motion was issued, but no torso motion seem to be in progress.")
            return False, False
        currentTorsoPosition = self.getJointPosition(params.torsoLiftJointName)
        if currentTorsoPosition is None:
            self.logWarnWithStatus("Cannot read current " + params.torsoLiftJointName + " yet from topic " + params.jointStateTopic + ", retrying...")
            return False, False
        error = abs(float(currentTorsoPosition) - float(self.torsoMotionTarget))
        if error <= self.torsoMotionTolerance:
            self.logInfoWithStatus(
                f"Torso target successfully reached: current={float(currentTorsoPosition):.4f}, target={float(self.torsoMotionTarget):.4f}, error={error:.4f}."
            )
            return True, True
        elapsedSeconds = (self.get_clock().now() - self.torsoMotionStartTime).nanoseconds * 1e-9
        if elapsedSeconds > self.torsoMotionTimeout:
            self.logErrorWithStatus(
                f"Torso motion timed out (timeout: {self.torsoMotionTimeout}s): current={float(currentTorsoPosition):.4f}, "
                f"target={float(self.torsoMotionTarget):.4f}, error={error:.4f}."
            )
            return True, False
        self.logInfoWithStatus(
            f"Waiting torso motion complention: current={float(currentTorsoPosition):.4f}, "
            f"target={float(self.torsoMotionTarget):.4f}, error={error:.4f}."
        )
        return False, False

    def publishHeadTiltCommand(self):
        # This method can be used to publish a command to control the tilt-position of the Tiago head,
        # and it is used (in this Task3 FSM) to tilt the head correctly in order to detect the cubes on the pick location table.
        # This is literally the very same method used in the Task2 FSM Node
        current_head_pan = self.getJointPosition(params.panHeadJointName)
        if current_head_pan is None:
            self.logWarnWithStatus("Cannot read current " + params.panHeadJointName + " yet from topic " + params.jointStateTopic + ", retrying...")
            return False
        target_head_tilt = float(params.headTiltDuringPickAndPlace)
        msg = JointTrajectory()
        msg.joint_names = [params.panHeadJointName, params.tiltHeadJointName]
        point = JointTrajectoryPoint()
        point.positions = [current_head_pan, target_head_tilt]
        point.time_from_start = Duration(sec = 2) # Imposed time duration for the head movement
        msg.points.append(point)
        self.headCommandPublisher.publish(msg)
        self.logInfoWithStatus(f"Published head tilt command: {params.panHeadJointName}={current_head_pan:.3f}, {params.tiltHeadJointName}={target_head_tilt:.3f}.")
        return True
    
    def broadcastTF(self, pose: Pose, parentFrameID: str, frameID: str):
        # This method simply broadcasts a TF transform from parentFrameID to frameID
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parentFrameID
        msg.child_frame_id = frameID
        msg.transform.translation.x = float(pose.position.x)
        msg.transform.translation.y = float(pose.position.y)
        msg.transform.translation.z = float(pose.position.z)
        msg.transform.rotation.x = float(pose.orientation.x)
        msg.transform.rotation.y = float(pose.orientation.y)
        msg.transform.rotation.z = float(pose.orientation.z)
        msg.transform.rotation.w = float(pose.orientation.w)
        self.tfBroadcaster.sendTransform(msg)

    def handle_initCubesTransportation(self):
        # In this state, the FSM carries out all the initialization operations needed to be done BEFORE starting the transportation of cubes.
        # (1) Store the actual robot pose (that will be considered as the "initial" one and that will be used to return to it after the whole transportation process)
        # (2) Tilt the Tiago head to a proper position in order to detect the cubes on the pick location table
        # (3) Load of the results of the cubes discovery (performed in Task2) from the related YAML files (indeed, the pick and place locations in the map)
        if self.initialRobotPose is None: # If None, that means that we are still missing the initial robot pose stored
            # (1) Store the initial pose
            # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
            if self.lastAMCLMsg is None:
                self.logWarnWithStatus("Waiting for an AMCL pose message to be available to store the initial robot pose before the cubes transportation...")
            else:
                self.initialRobotPose = PoseStamped()
                self.initialRobotPose.header.frame_id = self.lastAMCLMsg.header.frame_id
                self.initialRobotPose.header.stamp = self.get_clock().now().to_msg()
                self.initialRobotPose.pose = self.lastAMCLMsg.pose.pose
                self.logInfoWithStatus(
                    "Successfully stored initial robot pose, now tilting Tiago head:"
                    f"frame={self.initialRobotPose.header.frame_id}, "
                    f"x={self.initialRobotPose.pose.position.x:.3f}, "
                    f"y={self.initialRobotPose.pose.position.y:.3f}."
                )
        # (2) Tilt the Tiago head to a proper position in order to detect the cubes on the pick location table
        # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
        elif not self.tiltHeadSent:
            if self.publishHeadTiltCommand(): self.tiltHeadSent = True
        else:
            current_head_tilt = self.getJointPosition(params.tiltHeadJointName)
            if current_head_tilt is None:
                self.logInfoWithStatus("Waiting for a value for " + params.tiltHeadJointName + " from " + params.jointStateTopic + "...")
                return
            target_head_tilt = float(params.headTiltDuringPickAndPlace)
            head_tilt_error = abs(current_head_tilt - target_head_tilt)
            if head_tilt_error > self.headTiltTolerance:
                self.logInfoWithStatus(f"Waiting head tilt: current={current_head_tilt:.3f}, target={target_head_tilt:.3f}, error={head_tilt_error:.3f}.")
            else:
                self.logInfoWithStatus(f"Head tilt done: current={current_head_tilt:.3f}, target={target_head_tilt:.3f}, error={head_tilt_error:.3f}.")
                # (3) Load discovery results (the load is attempted only ONCE; TODO: implement a limited-in-amount retry mechanism in case of failure)
                self.logInfoWithStatus("Loading Task2 discovery results from YAML files...")
                try:
                    self.pickLocation, self.placeLocation = discoveryLoader.loadTask2LocationEstimates(self.savedMapPath)
                    # Note that the TF transforms broadcasting is not strictly required for the correct execution of the Task3 FSM,
                    # but it is useful for debugging and visualization purposes (e.g. in RViz) and therefore it has been performed.
                    self.broadcastTF(self.pickLocation.pose, self.pickLocation.frame_id, "task2_pick_location_estimate")
                    self.broadcastTF(self.placeLocation.pose, self.placeLocation.frame_id, "task2_place_location_estimate")
                    self.logInfoWithStatus("Task2 discovery results loaded successfully!")
                    self.currentFSMstate = Task3FSMState.NAVIGATE_TO_PICK_PLATFORM
                except Exception as e:
                    self.logErrorWithStatus(f"Failed to load Task2 discovery results, now shutting down the FSM Task3 Node. Error: {e}")
                    self.currentFSMstate = Task3FSMState.FINISH
                    return

    def rotateVectorByQuaternion(self, vector, qx: float, qy: float, qz: float, qw: float):
        # A simple utility method that rotates a 3D vector by a quaternion (qx, qy, qz, qw)
        vx, vy, vz = vector
        ix =  qw * vx + qy * vz - qz * vy
        iy =  qw * vy + qz * vx - qx * vz
        iz =  qw * vz + qx * vy - qy * vx
        iw = -qx * vx - qy * vy - qz * vz
        rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
        ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
        rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
        return (rx, ry, rz)
    
    def quaternionFromYaw(self, yaw: float):
        # A simple utility method that builds a quaternion from a yaw angle (in radians); the quaternion is returned as a dictionary form, with keys 'x', 'y', 'z', 'w'
        half_yaw = yaw * 0.5
        return {'x': 0.0, 'y': 0.0, 'z': math.sin(half_yaw), 'w': math.cos(half_yaw) }
    
    def multiplyQuaternionsAndNormalize(self, q1, q2):
        # A simple utility method that multiplies two quaternions and normalizes the result
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        q = [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        ]
        norm = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        if norm <= 1e-9: raise ValueError("Error while multiplying quaternions: the resulting quaternion has a norm too close to zero (norm = {:.6f})".format(norm))
        return [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm]
    
    def transformChangeReferenceFrame(self, arucoPoseEstimate: params.ArucoPoseEstimate, targetFrameID: str):
        # A simple utility method that transforms a given ArucoPoseEstimate to a different reference frame (indeed, targetFrameID)
        sourceFrameID = str(arucoPoseEstimate.frame_id)
        transform = self.tfBuffer.lookup_transform(
            targetFrameID,
            sourceFrameID,
            rclpy.time.Time(),
        )
        transPose = do_transform_pose(arucoPoseEstimate.pose, transform)
        return params.ArucoPoseEstimate(
            found=arucoPoseEstimate.found,
            marker_id=arucoPoseEstimate.marker_id,
            label=arucoPoseEstimate.label,
            frame_id=targetFrameID, # New parent frame!
            marker_frame=arucoPoseEstimate.marker_frame,
            sample_count=arucoPoseEstimate.sample_count,
            pose = copy.deepcopy(transPose)
        )


    
     #-----------------------------------------------------------------------------------------------

    def yaw_from_quaternion(self, qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    

    def pose_dict_to_pose_stamped(self, pose_dict: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(pose_dict['frame_id'])
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_dict['position']['x'])
        pose.pose.position.y = float(pose_dict['position']['y'])
        pose.pose.position.z = float(pose_dict['position']['z'])
        pose.pose.orientation.x = float(pose_dict['orientation']['x'])
        pose.pose.orientation.y = float(pose_dict['orientation']['y'])
        pose.pose.orientation.z = float(pose_dict['orientation']['z'])
        pose.pose.orientation.w = float(pose_dict['orientation']['w'])
        return pose

    def pose_stamped_to_pose_dict(self, pose: PoseStamped, marker_id: int, label: str) -> dict:
        return {
            'marker_id': int(marker_id),
            'label': str(label),
            'frame_id': str(pose.header.frame_id),
            'position': {
                'x': float(pose.pose.position.x),
                'y': float(pose.pose.position.y),
                'z': float(pose.pose.position.z),
            },
            'orientation': {
                'x': float(pose.pose.orientation.x),
                'y': float(pose.pose.orientation.y),
                'z': float(pose.pose.orientation.z),
                'w': float(pose.pose.orientation.w),
            },
        }

    def transform_pose_dict(self, pose_dict: dict, target_frame: str) -> dict:
        source_frame = str(pose_dict['frame_id'])
        poseStamped = self.pose_dict_to_pose_stamped(pose_dict)

        transform = self.tfBuffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
        )

        # In ROS 2 Humble, tf2_geometry_msgs.do_transform_pose() expects a
        # geometry_msgs/Pose, not a PoseStamped. Passing PoseStamped would make
        # the helper look for pose.position directly and fail.
        transformedPose = do_transform_pose(poseStamped.pose, transform)

        transformedPoseStamped = PoseStamped()
        transformedPoseStamped.header.frame_id = target_frame
        transformedPoseStamped.header.stamp = self.get_clock().now().to_msg()
        transformedPoseStamped.pose = transformedPose

        return self.pose_stamped_to_pose_dict(
            transformedPoseStamped,
            marker_id=int(pose_dict['marker_id']),
            label=str(pose_dict['label']),
        )
    
     #-----------------------------------------------------------------------------------------------

    def broadcast_pose_dict(self, pose_dict: dict, child_frame_id: str):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(pose_dict['frame_id'])
        msg.child_frame_id = child_frame_id
        msg.transform.translation.x = float(pose_dict['position']['x'])
        msg.transform.translation.y = float(pose_dict['position']['y'])
        msg.transform.translation.z = float(pose_dict['position']['z'])
        msg.transform.rotation.x = float(pose_dict['orientation']['x'])
        msg.transform.rotation.y = float(pose_dict['orientation']['y'])
        msg.transform.rotation.z = float(pose_dict['orientation']['z'])
        msg.transform.rotation.w = float(pose_dict['orientation']['w'])
        self.tfBroadcaster.sendTransform(msg)

    def buildPlatformApproachGoal(self, platformLocation: params.ArucoPoseEstimate, approachDistance: float) -> NavigateToPose.Goal:
        # This method builds a navigation goal to approach a given platform (pick or place) at a specified distance from it.
        markerX = float(platformLocation.position['x'])
        markerY = float(platformLocation.position['y'])
        markerZ = float(platformLocation.position['z'])
        markerQX = float(platformLocation.orientation['x'])
        markerQY = float(platformLocation.orientation['y'])
        markerQZ = float(platformLocation.orientation['z'])
        markerQW = float(platformLocation.orientation['w'])

        offsetX, offsetY, offsetZ = self.rotateVectorByQuaternion( # The rotation brings the offset vector from the marker frame to the map frame
            (0.0, 0.0, approachDistance), # Defined in the marker frame, so that the offset is along the marker's Z-axis (i.e. the normal to the marker plane)
            markerQX,
            markerQY,
            markerQZ,
            markerQW,
        )

        # Define the goal in the map reference frame
        goalX = markerX + offsetX
        goalY = markerY + offsetY
        goalZ = markerZ + offsetZ
        yawPointingToMarker = math.atan2(markerY - goalY, markerX - goalX)
        goalQuaternion = self.quaternionFromYaw(yawPointingToMarker)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = platformLocation.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(goalX)
        goal.pose.pose.position.y = float(goalY)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = goalQuaternion['x']
        goal.pose.pose.orientation.y = goalQuaternion['y']
        goal.pose.pose.orientation.z = goalQuaternion['z']
        goal.pose.pose.orientation.w = goalQuaternion['w']

        # Broadcasting the goal as a TF transform for visualization/debugging purposes (e.g. in RViz)
        self.broadcastTF(goal.pose.pose, f"task3_{platformLocation.label}_platform_approach") 
        return goal

    def handle_navigateToPickPlatform(self):
        # In this state, the FSM sends a navigation goal to the Nav2 stack in order to approach the pick platform
        # Note: at this stage, the currently selected sube to be transported is indeed ALREADY selected (and it's the one with index self.currentCubeIndex)
        if not self.pickPlatformNavigationSent:
            if not self.navigateToPoseActionClient.server_is_ready():
                self.logInfoWithStatus("Waiting for NavigateToPose action server...")
                return
            self.logInfoWithStatus("Sending to Nav2 a NavigateToPose goal to approach the pick platform.")
            pickGoal = self.buildPlatformApproachGoal(self.pickLocation, params.platformApproachDistance)
            self.pickPlatformNavigationSent = True
            self.pickPlatformNavigationDone = False
            self.pickPlatformNavigationSucceeded = False
            future = self.navigateToPoseActionClient.send_goal_async(pickGoal)
            future.add_done_callback(self.pickPlatformNavigationGoalResponseCallback)
        else:
            if not self.pickPlatformNavigationDone: return
            if not self.pickPlatformNavigationSucceeded:
                self.logErrorWithStatus("Navigation to pick platform failed, now retrying...")
                # TODO: implement a WatchDog mechanism to avoid retrying indefinitely
                self.pickPlatformNavigationSent = False
            else:
                self.logInfoWithStatus("Successfully navigated to the pick platform")
                self.collectCurrentCubeSamplesENABLER = True
                self.cubeDetectionWaitStartTime = None
                self.currentFSMstate = Task3FSMState.CUBE_DETECTION

    def pickPlatformNavigationGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: add explicit exception handling
        if not goalHandle.accepted:
            self.logErrorWithStatus("NavigateToPose goal (to navigate to pick platform approach) rejected by Nav2.")
            self.pickPlatformNavigationDone = True
            self.pickPlatformNavigationSucceeded = False
            return
        self.logInfoWithStatus("NavigateToPose goal (to navigate to pick platform approach) accepted by Nav2.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.pickPlatformNavigationResultCallback)

    def pickPlatformNavigationResultCallback(self, future):
        wrappedResult = future.result() # TODO: add explicit exception handling
        self.pickPlatformNavigationDone = True
        self.pickPlatformNavigationSucceeded = wrappedResult.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f"NavigateToPose goal (to navigate to pick platform approach) completed with status {wrappedResult.status}.")
        
    def handle_cubeDetection(self):
        # In this state, the FSM waits for a certain amount of time (defined by the parameter pickingWaitingTime) in order to collect enough samples
        # of the current cube marker before computing an estimate of its pose (it is supposed that the cube is in the current camera view).
        # If during this waiting period no samples are collected, then the collection is restarted from scratch (this is a very simple behavior).
        # Once the collection is completed, they are computed:
        # (1) the final pose estimate of the cube w.r.t. the map reference frame
        # (2) the final pose estimate of the cube w.r.t. the Tiago base_link reference frame
        # (3) the approach pose for the Tiago gripper to indeed approach the cube (again, w.r.t. the Tiago base_link reference frame)
        if self.cubeDetectionWaitStartTime is None:
            # Starting collection
            self.cubeDetectionWaitStartTime = self.get_clock().now()
            self.logInfoWithStatus(f"Now collecting marker {self.cubesData[self.currentCubeIndex].markerID} samples for {params.pickingWaitingTime:.2f} seconds...")
        else:
            # Verifying collection
            elapsedTime = (self.get_clock().now() - self.cubeDetectionWaitStartTime).nanoseconds * 1e-9
            if elapsedTime < float(params.pickingWaitingTime): return
            self.collectCurrentCubeSamplesENABLER = False
            if len(self.currentCubeSamples) == 0:
                self.logWarnWithStatus(
                    f"No samples at all collected for marker {self.cubesData[self.currentCubeIndex].markerID} in the waiting period {params.pickingWaitingTime:.2f}. Restarting collection."
                )
                self.cubesData[self.currentCubeIndex].arucoSamples.clear()
                self.collectCurrentCubeSamplesENABLER = True
                self.cubeDetectionWaitStartTime = None
                return
            # Computing the final pose estimate and storing it
            samples = self.cubesData[self.currentCubeIndex].arucoSamples
            position = self.computePositionsMedoid(samples)
            quaternion = self.computeOrientationsMedoid(samples)
            frameId = samples[0].frame_id if samples[0].frame_id else params.fatherReferenceFrame
            self.cubesData[self.currentCubeIndex].detectionWaitStartTime = self.cubeDetectionWaitStartTime
            self.cubeDetectionWaitStartTime = None
            # Saving the final pose estimate referenced w.r.t. the map reference frame
            self.cubesData[self.currentCubeIndex].finalPoseMap = params.ArucoPoseEstimate(
                found = True, # Whether at least one valid marker sample was collected.
                marker_id = self.cubesData[self.currentCubeIndex].markerID, # Numerical ID of the ArUco marker associated with this location.
                label = self.cubesData[self.currentCubeIndex].markerNickname, # Semantic role of the location, e.g. "pick" or "place".
                frame_id = frameId, # Reference frame in which the estimated pose is expressed, typically "map" ("map" is indeed used as a default value).
                marker_frame = self.cubesData[self.currentCubeIndex].markerInfo.markerFrame, # TF frame name associated with the specific ArUco marker.
                sample_count = len(samples), # Number of samples used to compute the estimate.
                position = {'x': float(position[0]), 'y': float(position[1]), 'z': float(position[2])}, # Estimated marker position {"x": ..., "y": ..., "z": ...}, if found.
                orientation = {'x': float(quaternion[0]), 'y': float(quaternion[1]), 'z': float(quaternion[2]), 'w': float(quaternion[3])} # Estimated marker orientation {"x": ..., "y": ..., "z": ..., "w": ...}, if found.
            )
            # Saving the final pose estimate referenced w.r.t. the Tiago base_link reference frame
            self.cubesData[self.currentCubeIndex].finalPoseBaseLink = self.transformChangeReferenceFrame(
                self.cubesData[self.currentCubeIndex].finalPoseMap,
                targetFrameID = params.tiagoBaseLinkReferenceFrame,
            )
            # Broadcasting the final pose estimates for visualization/debugging purposes (e.g. in RViz)
            self.broadcastTF(
                self.cubesData[self.currentCubeIndex].finalPoseMap.pose,
                parentFrameID = self.cubesData[self.currentCubeIndex].finalPoseMap.frame_id,
                frameID = f"task3_{self.cubesData[self.currentCubeIndex].markerNickname}_marker_pose_map"
            )
            self.broadcastTF(
                self.cubesData[self.currentCubeIndex].finalPoseBaseLink.pose,
                parentFrameID = self.cubesData[self.currentCubeIndex].finalPoseBaseLink.frame_id,
                frameID = f"task3_{self.cubesData[self.currentCubeIndex].markerNickname}_marker_pose_base_link"
            )
            self.logInfoWithStatus(
                f"Cube {self.cubesData[self.currentCubeIndex].markerNickname} pose estimated from "
                f"{len(self.currentCubeSamples)} samples and successfully stored (BOTH w.r.t. map/base_link reference frames)."
            )
            # Building up the approach pose for the Tiago gripper to indeed approach the cube (w.r.t. the Tiago base_link reference frame)
            cubePose = self.cubesData[self.currentCubeIndex].finalPoseBaseLink.pose # This is a Pose object
            approachPose = Pose()
            approachPose.position.x = cubePose.position.x
            approachPose.position.y = cubePose.position.y
            approachPose.position.z = cubePose.position.z + params.cubeApproachHeight
            # Note: exploiting a quaternion representing a pitch rotation of +90 degrees (in radians) around the Y-axis
            pitchPlus90Quaternion = [0.0, math.sin(math.pi / 4.0), 0.0, math.cos(math.pi / 4.0)]
            approachQuaternion = self.multiplyQuaternionsAndNormalize(
                [approachQuaternion.x, approachQuaternion.y, approachQuaternion.z, approachQuaternion.w],
                pitchPlus90Quaternion,
            )
            approachPose.orientation.x = approachQuaternion[0]
            approachPose.orientation.y = approachQuaternion[1]
            approachPose.orientation.z = approachQuaternion[2]
            approachPose.orientation.w = approachQuaternion[3]
            self.cubesData[self.currentCubeIndex].approachPose = approachPose
            # Broadcasting the approach pose for visualization/debugging purposes (e.g. in RViz)
            self.broadcastTF(
                approachPose,
                parentFrameID = self.cubesData[self.currentCubeIndex].finalPoseBaseLink.frame_id,
                frameID = f"task3_{self.cubesData[self.currentCubeIndex].markerNickname}_approach_pose_base_link"
            )
            self.logInfoWithStatus(
                f"Built arm approach pose for cube {self.cubesData[self.currentCubeIndex].markerNickname}: "
                f"position={self.currentCubeApproachPosition}, quat_xyzw={self.currentCubeApproachQuaternion}."
            )
            self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_PICKING_CONFIGURATION

    def handle_moveArmToPickingConfiguration(self):
        # In this state, the FSM exploits the TiagoArm Action Server in order to move the arm to a predefined picking configuration (joint positions).
        # Note that the "picking configuration" (defined in the jointspace) is NOT the approach pose for the gripper; it's instead a predefined configuration
        # to be firstly reached by the Tiago arm BEFORE moving the gripper to the approach pose (defined in the Cartesian space).
        if not self.tiagoArmActionClient.server_is_ready():
            self.logInfoWithStatus("Waiting for TiagoArm Action Server...")
            return
        goalMsg = TiagoArm.Goal()
        goalMsg.joint_positions = params.PICKING_JOINT_POSITIONS
        goalMsg.position_obj = []
        goalMsg.quat_xyzw_obj = []

        self.armPickingGoalDone = False
        self.armPickingGoalSucceeded = False

        self.get_logger().info("[MOVE_ARM_TO_PICKING_CONFIGURATION] Sending picking joint configuration to TiagoArm Action Server.")
        future = self.tiagoArmActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.armPickingFeedbackCallback,
        )
        future.add_done_callback(self.armPickingGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_ARM_TO_PICKING_CONFIGURATION

    def armPickingFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[MOVE_ARM_TO_PICKING_CONFIGURATION] Feedback: {feedback.current_state}")

    def armPickingGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[MOVE_ARM_TO_PICKING_CONFIGURATION] Goal rejected by TiagoArm Action Server.")
            self.armPickingGoalDone = True
            self.armPickingGoalSucceeded = False
            return

        self.get_logger().info("[MOVE_ARM_TO_PICKING_CONFIGURATION] Goal accepted by TiagoArm Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.armPickingResultCallback)

    def armPickingResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[MOVE_ARM_TO_PICKING_CONFIGURATION] Success: {result.message}")
            self.armPickingGoalDone = True
            self.armPickingGoalSucceeded = True
        else:
            self.get_logger().error(f"[MOVE_ARM_TO_PICKING_CONFIGURATION] Failed: {result.message}")
            self.armPickingGoalDone = True
            self.armPickingGoalSucceeded = False

    def handle_waitArmToPickingConfiguration(self):
        if not self.armPickingGoalDone:
            return

        if not self.armPickingGoalSucceeded:
            self.get_logger().error("[WAIT_ARM_TO_PICKING_CONFIGURATION] Arm picking configuration motion failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_ARM_TO_PICKING_CONFIGURATION] Arm reached picking configuration.")
        self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_CURRENT_CUBE_APPROACH

    def handle_moveArmToCurrentCubeApproach(self):
        if not self.tiagoArmActionClient.server_is_ready():
            self.get_logger().info("[MOVE_ARM_TO_CURRENT_CUBE_APPROACH] Waiting for TiagoArm Action Server...")
            return

        if self.currentCubeApproachPosition is None or self.currentCubeApproachQuaternion is None:
            self.get_logger().error("[MOVE_ARM_TO_CURRENT_CUBE_APPROACH] Missing approach pose.")
            self.shouldShutdown = True
            return

        goalMsg = TiagoArm.Goal()
        goalMsg.joint_positions = []
        goalMsg.position_obj = list(self.currentCubeApproachPosition)
        goalMsg.quat_xyzw_obj = list(self.currentCubeApproachQuaternion)

        self.armApproachGoalDone = False
        self.armApproachGoalSucceeded = False

        self.gripperOpenGoalDone = False
        self.gripperOpenGoalSucceeded = False

        self.gripperCloseGoalDone = False
        self.gripperCloseGoalSucceeded = False

        self.armGraspingGoalDone = False
        self.armGraspingGoalSucceeded = False

        self.armReturnApproachGoalDone = False
        self.armReturnApproachGoalSucceeded = False

        self.armTransportGoalDone = False
        self.armTransportGoalSucceeded = False

        self.get_logger().info("[MOVE_ARM_TO_CURRENT_CUBE_APPROACH] Sending pose goal to TiagoArm Action Server.")
        future = self.tiagoArmActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.armApproachFeedbackCallback,
        )
        future.add_done_callback(self.armApproachGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_ARM_TO_CURRENT_CUBE_APPROACH

    def armApproachFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[MOVE_ARM_TO_CURRENT_CUBE_APPROACH] Feedback: {feedback.current_state}")

    def armApproachGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[MOVE_ARM_TO_CURRENT_CUBE_APPROACH] Goal rejected by TiagoArm Action Server.")
            self.armApproachGoalDone = True
            self.armApproachGoalSucceeded = False
            return

        self.get_logger().info("[MOVE_ARM_TO_CURRENT_CUBE_APPROACH] Goal accepted by TiagoArm Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.armApproachResultCallback)

    def armApproachResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[MOVE_ARM_TO_CURRENT_CUBE_APPROACH] Success: {result.message}")
            self.armApproachGoalDone = True
            self.armApproachGoalSucceeded = True
        else:
            self.get_logger().error(f"[MOVE_ARM_TO_CURRENT_CUBE_APPROACH] Failed: {result.message}")
            self.armApproachGoalDone = True
            self.armApproachGoalSucceeded = False

    def handle_waitArmToCurrentCubeApproach(self):
        if not self.armApproachGoalDone:
            return

        if not self.armApproachGoalSucceeded:
            self.get_logger().error("[WAIT_ARM_TO_CURRENT_CUBE_APPROACH] Arm approach motion failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_ARM_TO_CURRENT_CUBE_APPROACH] Arm reached current cube approach pose.")
        self.currentFSMstate = Task3FSMState.OPEN_GRIPPER

    def handle_openGripper(self):
        if not self.tiagoGripperActionClient.server_is_ready():
            self.get_logger().info("[OPEN_GRIPPER] Waiting for TiagoGripper Action Server...")
            return

        goalMsg = TiagoGripper.Goal()
        goalMsg.open = True
        goalMsg.object_model_name = ""
        goalMsg.object_link_name = ""

        self.gripperOpenGoalDone = False
        self.gripperOpenGoalSucceeded = False

        self.get_logger().info("[OPEN_GRIPPER] Sending open goal to TiagoGripper Action Server.")
        future = self.tiagoGripperActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.gripperOpenFeedbackCallback,
        )
        future.add_done_callback(self.gripperOpenGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_GRIPPER_OPENED

    def gripperOpenFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[OPEN_GRIPPER] Feedback: {feedback.current_state}")

    def gripperOpenGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[OPEN_GRIPPER] Goal rejected by TiagoGripper Action Server.")
            self.gripperOpenGoalDone = True
            self.gripperOpenGoalSucceeded = False
            return

        self.get_logger().info("[OPEN_GRIPPER] Goal accepted by TiagoGripper Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.gripperOpenResultCallback)

    def gripperOpenResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[OPEN_GRIPPER] Success: {result.message}")
            self.gripperOpenGoalDone = True
            self.gripperOpenGoalSucceeded = True
        else:
            self.get_logger().error(f"[OPEN_GRIPPER] Failed: {result.message}")
            self.gripperOpenGoalDone = True
            self.gripperOpenGoalSucceeded = False

    def handle_waitGripperOpened(self):
        if not self.gripperOpenGoalDone:
            return

        if not self.gripperOpenGoalSucceeded:
            self.get_logger().error("[WAIT_GRIPPER_OPENED] Gripper opening failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_GRIPPER_OPENED] Gripper opened successfully.")
        self.currentFSMstate = Task3FSMState.BUILD_CURRENT_CUBE_GRASPING_POSE

    def handle_buildCurrentCubeGraspingPose(self):
        if self.currentCubeApproachPosition is None or self.currentCubeApproachQuaternion is None:
            self.get_logger().error("[BUILD_CURRENT_CUBE_GRASPING_POSE] Missing current cube approach pose.")
            self.shouldShutdown = True
            return

        self.currentCubeGraspingPosition = [
            float(self.currentCubeApproachPosition[0]),
            float(self.currentCubeApproachPosition[1]),
            float(self.currentCubeApproachPosition[2]) - float(nodesParameters.translationGrasping),
        ]
        self.currentCubeGraspingQuaternion = list(self.currentCubeApproachQuaternion)

        graspingPose = {
            'marker_id': self.currentCubeMarkerId,
            'label': f"cube_{self.currentCubeMarkerId}_grasping",
            'frame_id': 'base_link',
            'position': {
                'x': self.currentCubeGraspingPosition[0],
                'y': self.currentCubeGraspingPosition[1],
                'z': self.currentCubeGraspingPosition[2],
            },
            'orientation': {
                'x': self.currentCubeGraspingQuaternion[0],
                'y': self.currentCubeGraspingQuaternion[1],
                'z': self.currentCubeGraspingQuaternion[2],
                'w': self.currentCubeGraspingQuaternion[3],
            },
        }
        self.broadcast_pose_dict(
            graspingPose,
            f"task3_cube_{self.currentCubeMarkerId}_grasping_pose_base_link",
        )

        self.get_logger().info(
            f"[BUILD_CURRENT_CUBE_GRASPING_POSE] Built arm grasping pose for marker {self.currentCubeMarkerId}: "
            f"position={self.currentCubeGraspingPosition}, quat_xyzw={self.currentCubeGraspingQuaternion}, "
            f"translationGrasping={float(nodesParameters.translationGrasping):.3f}."
        )
        self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_CURRENT_CUBE_GRASPING

    def handle_moveArmToCurrentCubeGrasping(self):
        currentTorsoPosition = self.getJointPosition("torso_lift_joint")
        if currentTorsoPosition is None:
            self.get_logger().info(
                "[MOVE_ARM_TO_CURRENT_CUBE_GRASPING] Waiting for torso_lift_joint from /joint_states..."
            )
            return

        self.torsoApproachJointPosition = float(currentTorsoPosition)

        requestedDescent = float(nodesParameters.translationGrasping)
        availableDescent = max(0.0, float(currentTorsoPosition) - float(self.torsoMinSafePosition))
        effectiveDescent = min(requestedDescent, availableDescent)

        if effectiveDescent <= 1e-6:
            self.get_logger().error(
                f"[MOVE_ARM_TO_CURRENT_CUBE_GRASPING] No valid torso descent available: "
                f"current_torso={float(currentTorsoPosition):.4f}, "
                f"torso_min_safe={float(self.torsoMinSafePosition):.4f}."
            )
            self.shouldShutdown = True
            return

        if effectiveDescent < requestedDescent:
            self.get_logger().warn(
                f"[MOVE_ARM_TO_CURRENT_CUBE_GRASPING] Requested descent {requestedDescent:.4f} exceeds "
                f"available safe torso descent {availableDescent:.4f}. "
                f"Using reduced descent {effectiveDescent:.4f} for marker {self.currentCubeMarkerId}."
            )

        self.currentCubeEffectiveGraspingTranslation = float(effectiveDescent)
        self.cubeEffectiveGraspingTranslations[int(self.currentCubeMarkerId)] = float(effectiveDescent)

        if self.currentCubeApproachPosition is not None:
            self.currentCubeGraspingPosition = [
                float(self.currentCubeApproachPosition[0]),
                float(self.currentCubeApproachPosition[1]),
                float(self.currentCubeApproachPosition[2]) - float(effectiveDescent),
            ]

        targetTorsoPosition = float(currentTorsoPosition) - float(effectiveDescent)

        self.armGraspingGoalDone = False
        self.armGraspingGoalSucceeded = False
        self.publishTorsoliftCommand(
            targetTorsoPosition,
            "MOVE_ARM_TO_CURRENT_CUBE_GRASPING",
        )
        self.currentFSMstate = Task3FSMState.WAIT_ARM_TO_CURRENT_CUBE_GRASPING

    def armGraspingFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[MOVE_ARM_TO_CURRENT_CUBE_GRASPING] Feedback: {feedback.current_state}")

    def armGraspingGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[MOVE_ARM_TO_CURRENT_CUBE_GRASPING] Goal rejected by TiagoArm Action Server.")
            self.armGraspingGoalDone = True
            self.armGraspingGoalSucceeded = False
            return

        self.get_logger().info("[MOVE_ARM_TO_CURRENT_CUBE_GRASPING] Goal accepted by TiagoArm Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.armGraspingResultCallback)

    def armGraspingResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[MOVE_ARM_TO_CURRENT_CUBE_GRASPING] Success: {result.message}")
            self.armGraspingGoalDone = True
            self.armGraspingGoalSucceeded = True
        else:
            self.get_logger().error(f"[MOVE_ARM_TO_CURRENT_CUBE_GRASPING] Failed: {result.message}")
            self.armGraspingGoalDone = True
            self.armGraspingGoalSucceeded = False

    def handle_waitArmToCurrentCubeGrasping(self):
        done, succeeded = self.isTorsoMotionCompleted("WAIT_ARM_TO_CURRENT_CUBE_GRASPING")
        if not done:
            return

        self.torsoMotionTarget = None
        self.torsoMotionStartTime = None
        self.armGraspingGoalDone = True
        self.armGraspingGoalSucceeded = succeeded

        if not succeeded:
            self.get_logger().error("[WAIT_ARM_TO_CURRENT_CUBE_GRASPING] Torso descent to grasping pose failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_ARM_TO_CURRENT_CUBE_GRASPING] Torso descent completed. Closing gripper.")
        self.currentFSMstate = Task3FSMState.CLOSE_GRIPPER

    def handle_closeGripper(self):
        if not self.tiagoGripperActionClient.server_is_ready():
            self.get_logger().info("[CLOSE_GRIPPER] Waiting for TiagoGripper Action Server...")
            return

        goalMsg = TiagoGripper.Goal()
        goalMsg.open = False
        # def get_current_cube_gazebo_model_name(self):
        #return f"aruco_cube_exam_id{int(self.currentCubeMarkerId)}"

    #def get_current_cube_gazebo_link_name(self):
     #   return "link"
        goalMsg.object_model_name = self.get_current_cube_gazebo_model_name()
        goalMsg.object_link_name = self.get_current_cube_gazebo_link_name()

        self.gripperCloseGoalDone = False
        self.gripperCloseGoalSucceeded = False

        self.get_logger().info("[CLOSE_GRIPPER] Sending close goal to TiagoGripper Action Server.")
        future = self.tiagoGripperActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.gripperCloseFeedbackCallback,
        )
        future.add_done_callback(self.gripperCloseGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_GRIPPER_CLOSED

    def gripperCloseFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[CLOSE_GRIPPER] Feedback: {feedback.current_state}")

    def gripperCloseGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[CLOSE_GRIPPER] Goal rejected by TiagoGripper Action Server.")
            self.gripperCloseGoalDone = True
            self.gripperCloseGoalSucceeded = False
            return

        self.get_logger().info("[CLOSE_GRIPPER] Goal accepted by TiagoGripper Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.gripperCloseResultCallback)

    def gripperCloseResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[CLOSE_GRIPPER] Success: {result.message}")
            self.gripperCloseGoalDone = True
            self.gripperCloseGoalSucceeded = True
        else:
            self.get_logger().error(f"[CLOSE_GRIPPER] Failed: {result.message}")
            self.gripperCloseGoalDone = True
            self.gripperCloseGoalSucceeded = False

    def handle_waitGripperClosed(self):
        if not self.gripperCloseGoalDone:
            return

        if not self.gripperCloseGoalSucceeded:
            self.get_logger().error("[WAIT_GRIPPER_CLOSED] Gripper closing failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_GRIPPER_CLOSED] Gripper closed successfully. Returning to approach pose.")
        self.currentFSMstate = Task3FSMState.MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH

    def handle_moveArmBackToCurrentCubeApproach(self):
        if self.torsoApproachJointPosition is None:
            self.get_logger().error(
                "[MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH] Missing stored torso approach position."
            )
            self.shouldShutdown = True
            return

        self.armReturnApproachGoalDone = False
        self.armReturnApproachGoalSucceeded = False
        self.publishTorsoliftCommand(
            float(self.torsoApproachJointPosition),
            "MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH",
        )
        self.currentFSMstate = Task3FSMState.WAIT_ARM_BACK_TO_CURRENT_CUBE_APPROACH

    def armReturnApproachFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH] Feedback: {feedback.current_state}")

    def armReturnApproachGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH] Goal rejected by TiagoArm Action Server.")
            self.armReturnApproachGoalDone = True
            self.armReturnApproachGoalSucceeded = False
            return

        self.get_logger().info("[MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH] Goal accepted by TiagoArm Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.armReturnApproachResultCallback)

    def armReturnApproachResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH] Success: {result.message}")
            self.armReturnApproachGoalDone = True
            self.armReturnApproachGoalSucceeded = True
        else:
            self.get_logger().error(f"[MOVE_ARM_BACK_TO_CURRENT_CUBE_APPROACH] Failed: {result.message}")
            self.armReturnApproachGoalDone = True
            self.armReturnApproachGoalSucceeded = False

    def handle_waitArmBackToCurrentCubeApproach(self):
        done, succeeded = self.isTorsoMotionCompleted("WAIT_ARM_BACK_TO_CURRENT_CUBE_APPROACH")
        if not done:
            return

        self.torsoMotionTarget = None
        self.torsoMotionStartTime = None
        self.armReturnApproachGoalDone = True
        self.armReturnApproachGoalSucceeded = succeeded

        if not succeeded:
            self.get_logger().error("[WAIT_ARM_BACK_TO_CURRENT_CUBE_APPROACH] Torso lift back to approach pose failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_ARM_BACK_TO_CURRENT_CUBE_APPROACH] Torso returned to approach height. Moving to transport configuration.")
        self.nextStateAfterTransportConfiguration = Task3FSMState.NAVIGATE_TO_PLACE_PLATFORM
        self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_TRANSPORT_CONFIGURATION

    def handle_moveArmToTransportConfiguration(self):
        if not self.tiagoArmActionClient.server_is_ready():
            self.get_logger().info("[MOVE_ARM_TO_TRANSPORT_CONFIGURATION] Waiting for TiagoArm Action Server...")
            return

        goalMsg = TiagoArm.Goal()
        goalMsg.joint_positions = nodesParameters.PICKING_JOINT_POSITIONS
        goalMsg.position_obj = []
        goalMsg.quat_xyzw_obj = []

        self.armTransportGoalDone = False
        self.armTransportGoalSucceeded = False

        self.get_logger().info("[MOVE_ARM_TO_TRANSPORT_CONFIGURATION] Sending transport joint configuration to TiagoArm Action Server.")
        future = self.tiagoArmActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.armTransportFeedbackCallback,
        )
        future.add_done_callback(self.armTransportGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_ARM_TO_TRANSPORT_CONFIGURATION

    def armTransportFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[MOVE_ARM_TO_TRANSPORT_CONFIGURATION] Feedback: {feedback.current_state}")

    def armTransportGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[MOVE_ARM_TO_TRANSPORT_CONFIGURATION] Goal rejected by TiagoArm Action Server.")
            self.armTransportGoalDone = True
            self.armTransportGoalSucceeded = False
            return

        self.get_logger().info("[MOVE_ARM_TO_TRANSPORT_CONFIGURATION] Goal accepted by TiagoArm Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.armTransportResultCallback)

    def armTransportResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[MOVE_ARM_TO_TRANSPORT_CONFIGURATION] Success: {result.message}")
            self.armTransportGoalDone = True
            self.armTransportGoalSucceeded = True
        else:
            self.get_logger().error(f"[MOVE_ARM_TO_TRANSPORT_CONFIGURATION] Failed: {result.message}")
            self.armTransportGoalDone = True
            self.armTransportGoalSucceeded = False

    def handle_waitArmToTransportConfiguration(self):
        if not self.armTransportGoalDone:
            return

        if not self.armTransportGoalSucceeded:
            self.get_logger().error("[WAIT_ARM_TO_TRANSPORT_CONFIGURATION] Arm transport configuration motion failed.")
            self.shouldShutdown = True
            return

        nextState = self.nextStateAfterTransportConfiguration
        self.nextStateAfterTransportConfiguration = None

        if nextState is None:
            self.get_logger().info("[WAIT_ARM_TO_TRANSPORT_CONFIGURATION] Arm reached transport configuration. No next state set. Finishing.")
            self.currentFSMstate = Task3FSMState.FINISH
            return

        self.get_logger().info(
            f"[WAIT_ARM_TO_TRANSPORT_CONFIGURATION] Arm reached transport configuration. Moving to {nextState.name}."
        )
        self.currentFSMstate = nextState


    def handle_navigateToPlacePlatform(self):
        if not self.navigateToPoseActionClient.server_is_ready():
            self.get_logger().info("[NAVIGATE_TO_PLACE_PLATFORM] Waiting for NavigateToPose action server...")
            return

        try:
            placeGoal = self.buildPlatformApproachGoal(
                self.task2DiscoveryResults.place,
                approachDistance=float(nodesParameters.platformApproachDistance),
            )
        except Exception as e:
            self.get_logger().error(f"[NAVIGATE_TO_PLACE_PLATFORM] Failed to build place platform goal: {e}")
            self.shouldShutdown = True
            return

        self.placePlatformNavigationDone = False
        self.placePlatformNavigationSucceeded = False
        self.placePlatformNavigationSent = True

        self.get_logger().info("[NAVIGATE_TO_PLACE_PLATFORM] Sending place platform approach goal to Nav2.")
        future = self.navigateToPoseActionClient.send_goal_async(placeGoal)
        future.add_done_callback(self.placePlatformNavigationGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_PLACE_PLATFORM_NAVIGATION

    def placePlatformNavigationGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[NAVIGATE_TO_PLACE_PLATFORM] Goal rejected by Nav2.")
            self.placePlatformNavigationDone = True
            self.placePlatformNavigationSucceeded = False
            return

        self.get_logger().info("[NAVIGATE_TO_PLACE_PLATFORM] Goal accepted by Nav2.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.placePlatformNavigationResultCallback)

    def placePlatformNavigationResultCallback(self, future):
        wrappedResult = future.result()  # TODO: add explicit exception handling.
        self.placePlatformNavigationDone = True
        self.placePlatformNavigationSucceeded = wrappedResult.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f"[NAVIGATE_TO_PLACE_PLATFORM] Completed with status {wrappedResult.status}.")

    def handle_waitPlacePlatformNavigation(self):
        if not self.placePlatformNavigationDone:
            return

        if not self.placePlatformNavigationSucceeded:
            self.get_logger().error("[WAIT_PLACE_PLATFORM_NAVIGATION] Place platform navigation failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_PLACE_PLATFORM_NAVIGATION] Place platform reached. Moving arm to saved relative approach pose.")
        self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_PLACE_APPROACH

    def handle_moveArmToPlaceApproach(self):
        if not self.tiagoArmActionClient.server_is_ready():
            self.get_logger().info("[MOVE_ARM_TO_PLACE_APPROACH] Waiting for TiagoArm Action Server...")
            return

        savedApproachPose = self.cubeApproachPosesBaseLink.get(int(self.currentCubeMarkerId))
        if savedApproachPose is None:
            self.get_logger().error(
                f"[MOVE_ARM_TO_PLACE_APPROACH] Missing saved base_link approach pose for marker {self.currentCubeMarkerId}."
            )
            self.shouldShutdown = True
            return

        goalMsg = TiagoArm.Goal()
        goalMsg.joint_positions = []
        goalMsg.position_obj = list(savedApproachPose['position'])
        goalMsg.quat_xyzw_obj = list(savedApproachPose['quaternion'])

        self.armPlaceApproachGoalDone = False
        self.armPlaceApproachGoalSucceeded = False

        self.get_logger().info(
            f"[MOVE_ARM_TO_PLACE_APPROACH] Sending saved relative approach pose to TiagoArm Action Server: "
            f"position={goalMsg.position_obj}, quat_xyzw={goalMsg.quat_xyzw_obj}."
        )
        future = self.tiagoArmActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.armPlaceApproachFeedbackCallback,
        )
        future.add_done_callback(self.armPlaceApproachGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_ARM_TO_PLACE_APPROACH

    def armPlaceApproachFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[MOVE_ARM_TO_PLACE_APPROACH] Feedback: {feedback.current_state}")

    def armPlaceApproachGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[MOVE_ARM_TO_PLACE_APPROACH] Goal rejected by TiagoArm Action Server.")
            self.armPlaceApproachGoalDone = True
            self.armPlaceApproachGoalSucceeded = False
            return

        self.get_logger().info("[MOVE_ARM_TO_PLACE_APPROACH] Goal accepted by TiagoArm Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.armPlaceApproachResultCallback)

    def armPlaceApproachResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[MOVE_ARM_TO_PLACE_APPROACH] Success: {result.message}")
            self.armPlaceApproachGoalDone = True
            self.armPlaceApproachGoalSucceeded = True
        else:
            self.get_logger().error(f"[MOVE_ARM_TO_PLACE_APPROACH] Failed: {result.message}")
            self.armPlaceApproachGoalDone = True
            self.armPlaceApproachGoalSucceeded = False

    def handle_waitArmToPlaceApproach(self):
        if not self.armPlaceApproachGoalDone:
            return

        if not self.armPlaceApproachGoalSucceeded:
            self.get_logger().error("[WAIT_ARM_TO_PLACE_APPROACH] Arm motion to place approach pose failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_ARM_TO_PLACE_APPROACH] Arm reached place approach pose. Descending torso to place.")
        self.currentFSMstate = Task3FSMState.MOVE_TORSO_TO_PLACE

    def handle_moveTorsoToPlace(self):
        currentTorsoPosition = self.getJointPosition("torso_lift_joint")
        if currentTorsoPosition is None:
            self.get_logger().info("[MOVE_TORSO_TO_PLACE] Waiting for torso_lift_joint from /joint_states...")
            return

        self.torsoApproachJointPosition = float(currentTorsoPosition)

        effectiveDescent = self.cubeEffectiveGraspingTranslations.get(int(self.currentCubeMarkerId))
        
        if effectiveDescent is None:
            self.get_logger().error(
                f"[MOVE_TORSO_TO_PLACE] Missing effective grasping descent for marker {self.currentCubeMarkerId}."
            )
            self.shouldShutdown = True
            return

        placingDescent = max(
            0.0,
            float(effectiveDescent) - float(nodesParameters.safetyPlacingTranslation)
        )

        targetTorsoPosition = float(currentTorsoPosition) - placingDescent
                

        if targetTorsoPosition < float(self.torsoMinSafePosition) - 1e-6:
            self.get_logger().error(
                f"[MOVE_TORSO_TO_PLACE] Cannot reuse effective descent {float(effectiveDescent):.4f} for marker "
                f"{self.currentCubeMarkerId}: current_torso={float(currentTorsoPosition):.4f}, "
                f"target={targetTorsoPosition:.4f}, torso_min_safe={float(self.torsoMinSafePosition):.4f}."
            )
            self.shouldShutdown = True
            return

        self.get_logger().info(
            f"[MOVE_TORSO_TO_PLACE] Reusing effective grasping descent {float(effectiveDescent):.4f} "
            f"for marker {self.currentCubeMarkerId}."
        )
        self.publishTorsoliftCommand(targetTorsoPosition, "MOVE_TORSO_TO_PLACE")
        self.currentFSMstate = Task3FSMState.WAIT_TORSO_TO_PLACE

    def handle_waitTorsoToPlace(self):
        done, succeeded = self.isTorsoMotionCompleted("WAIT_TORSO_TO_PLACE")
        if not done:
            return

        self.torsoMotionTarget = None
        self.torsoMotionStartTime = None

        if not succeeded:
            self.get_logger().error("[WAIT_TORSO_TO_PLACE] Torso descent to place pose failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_TORSO_TO_PLACE] Torso descent completed. Opening gripper to release cube.")
        self.currentFSMstate = Task3FSMState.OPEN_GRIPPER_FOR_PLACE

    def handle_openGripperForPlace(self):
        if not self.tiagoGripperActionClient.server_is_ready():
            self.get_logger().info("[OPEN_GRIPPER_FOR_PLACE] Waiting for TiagoGripper Action Server...")
            return

        goalMsg = TiagoGripper.Goal()
        goalMsg.open = True
        goalMsg.object_model_name = ""
        goalMsg.object_link_name = ""

        self.gripperOpenGoalDone = False
        self.gripperOpenGoalSucceeded = False

        self.get_logger().info("[OPEN_GRIPPER_FOR_PLACE] Sending open/detach goal to TiagoGripper Action Server.")
        future = self.tiagoGripperActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.gripperOpenFeedbackCallback,
        )
        future.add_done_callback(self.gripperOpenGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_GRIPPER_OPENED_FOR_PLACE

    def handle_waitGripperOpenedForPlace(self):
        if not self.gripperOpenGoalDone:
            return

        if not self.gripperOpenGoalSucceeded:
            self.get_logger().error("[WAIT_GRIPPER_OPENED_FOR_PLACE] Gripper opening/release failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_GRIPPER_OPENED_FOR_PLACE] Cube released successfully. Lifting torso back from place.")
        self.currentFSMstate = Task3FSMState.MOVE_TORSO_BACK_FROM_PLACE

    def handle_moveTorsoBackFromPlace(self):
        if self.torsoApproachJointPosition is None:
            self.get_logger().error("[MOVE_TORSO_BACK_FROM_PLACE] Missing stored torso approach position.")
            self.shouldShutdown = True
            return

        self.publishTorsoliftCommand(float(self.torsoApproachJointPosition), "MOVE_TORSO_BACK_FROM_PLACE")
        self.currentFSMstate = Task3FSMState.WAIT_TORSO_BACK_FROM_PLACE

    def handle_waitTorsoBackFromPlace(self):
        done, succeeded = self.isTorsoMotionCompleted("WAIT_TORSO_BACK_FROM_PLACE")
        if not done:
            return

        self.torsoMotionTarget = None
        self.torsoMotionStartTime = None

        if not succeeded:
            self.get_logger().error("[WAIT_TORSO_BACK_FROM_PLACE] Torso lift back from place failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_TORSO_BACK_FROM_PLACE] Torso returned from place. Moving to transport configuration.")
        self.nextStateAfterTransportConfiguration = Task3FSMState.CLOSE_GRIPPER_AFTER_PLACE
        self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_TRANSPORT_CONFIGURATION

    def handle_closeGripperAfterPlace(self):
        if not self.tiagoGripperActionClient.server_is_ready():
            self.get_logger().info("[CLOSE_GRIPPER_AFTER_PLACE] Waiting for TiagoGripper Action Server...")
            return

        goalMsg = TiagoGripper.Goal()
        goalMsg.open = False
        goalMsg.object_model_name = ""
        goalMsg.object_link_name = ""

        self.gripperCloseGoalDone = False
        self.gripperCloseGoalSucceeded = False

        self.get_logger().info("[CLOSE_GRIPPER_AFTER_PLACE] Sending close goal to TiagoGripper Action Server without LinkAttacher target.")
        future = self.tiagoGripperActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.gripperCloseFeedbackCallback,
        )
        future.add_done_callback(self.gripperCloseGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_GRIPPER_CLOSED_AFTER_PLACE

    def handle_waitGripperClosedAfterPlace(self):
        if not self.gripperCloseGoalDone:
            return

        if not self.gripperCloseGoalSucceeded:
            self.get_logger().error("[WAIT_GRIPPER_CLOSED_AFTER_PLACE] Gripper closing after place failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_GRIPPER_CLOSED_AFTER_PLACE] Gripper closed after place. Moving arm to home configuration.")
        self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_HOME_AFTER_PLACE

    def handle_moveArmToHomeAfterPlace(self):
        if not self.tiagoArmActionClient.server_is_ready():
            self.get_logger().info("[MOVE_ARM_TO_HOME_AFTER_PLACE] Waiting for TiagoArm Action Server...")
            return

        goalMsg = TiagoArm.Goal()
        goalMsg.joint_positions = nodesParameters.HOME_JOINT_POSITIONS
        goalMsg.position_obj = []
        goalMsg.quat_xyzw_obj = []

        self.armHomeAfterPlaceGoalDone = False
        self.armHomeAfterPlaceGoalSucceeded = False

        self.get_logger().info("[MOVE_ARM_TO_HOME_AFTER_PLACE] Sending home joint configuration to TiagoArm Action Server.")
        future = self.tiagoArmActionClient.send_goal_async(
            goalMsg,
            feedback_callback=self.armHomeAfterPlaceFeedbackCallback,
        )
        future.add_done_callback(self.armHomeAfterPlaceGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_ARM_TO_HOME_AFTER_PLACE

    def armHomeAfterPlaceFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[MOVE_ARM_TO_HOME_AFTER_PLACE] Feedback: {feedback.current_state}")

    def armHomeAfterPlaceGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[MOVE_ARM_TO_HOME_AFTER_PLACE] Goal rejected by TiagoArm Action Server.")
            self.armHomeAfterPlaceGoalDone = True
            self.armHomeAfterPlaceGoalSucceeded = False
            return

        self.get_logger().info("[MOVE_ARM_TO_HOME_AFTER_PLACE] Goal accepted by TiagoArm Action Server.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.armHomeAfterPlaceResultCallback)

    def armHomeAfterPlaceResultCallback(self, future):
        result = future.result().result  # TODO: add explicit exception handling.
        if result.success:
            self.get_logger().info(f"[MOVE_ARM_TO_HOME_AFTER_PLACE] Success: {result.message}")
            self.armHomeAfterPlaceGoalDone = True
            self.armHomeAfterPlaceGoalSucceeded = True
        else:
            self.get_logger().error(f"[MOVE_ARM_TO_HOME_AFTER_PLACE] Failed: {result.message}")
            self.armHomeAfterPlaceGoalDone = True
            self.armHomeAfterPlaceGoalSucceeded = False

    def handle_waitArmToHomeAfterPlace(self):
        if not self.armHomeAfterPlaceGoalDone:
            return

        if not self.armHomeAfterPlaceGoalSucceeded:
            self.get_logger().error("[WAIT_ARM_TO_HOME_AFTER_PLACE] Arm home-after-place motion failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_ARM_TO_HOME_AFTER_PLACE] Arm reached home configuration. Advancing to next cube or finishing.")
        self.currentFSMstate = Task3FSMState.ADVANCE_TO_NEXT_CUBE

    def handle_advanceToNextCube(self):
        self.currentCubeIndex += 1
        if self.currentCubeIndex >= len(self.cubeMarkerIdsToMove):
            self.get_logger().info("[ADVANCE_TO_NEXT_CUBE] No more cubes to move. Returning to initial robot pose.")
            self.currentFSMstate = Task3FSMState.RETURN_TO_INITIAL_POSE
            return

        self.currentCubeMarkerId = self.cubeMarkerIdsToMove[self.currentCubeIndex]
        self.currentCubeSamples.clear()
        self.collectCurrentCubeSamplesENABLER = False
        self.cubeDetectionWaitStartTime = None
        self.currentCubeFinalPoseMap = None
        self.currentCubeFinalPoseBaseLink = None
        self.currentCubeApproachPosition = None
        self.currentCubeApproachQuaternion = None
        self.currentCubeGraspingPosition = None
        self.currentCubeGraspingQuaternion = None
        self.currentCubeEffectiveGraspingTranslation = None
        self.torsoApproachJointPosition = None
        self.armHomeAfterPlaceGoalDone = False
        self.armHomeAfterPlaceGoalSucceeded = False
        self.gripperCloseGoalDone = False
        self.gripperCloseGoalSucceeded = False

        self.get_logger().info(
            f"[ADVANCE_TO_NEXT_CUBE] Moving to next cube marker {self.currentCubeMarkerId}. Returning to pick platform navigation."
        )
        self.currentFSMstate = Task3FSMState.NAVIGATE_TO_PICK_PLATFORM

    def handle_returnToInitialPose(self):
        if self.initialRobotPose is None:
            self.get_logger().warn(
                "[RETURN_TO_INITIAL_POSE] Initial robot pose was not stored. Skipping return navigation."
            )
            self.currentFSMstate = Task3FSMState.FINISH
            return

        if not self.navigateToPoseActionClient.server_is_ready():
            self.get_logger().info("[RETURN_TO_INITIAL_POSE] Waiting for NavigateToPose action server...")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.initialRobotPose.header.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = self.initialRobotPose.pose

        self.returnToInitialPoseDone = False
        self.returnToInitialPoseSucceeded = False

        self.get_logger().info(
            "[RETURN_TO_INITIAL_POSE] Sending initial robot pose goal to Nav2: "
            f"frame={goal.pose.header.frame_id}, "
            f"x={goal.pose.pose.position.x:.3f}, "
            f"y={goal.pose.pose.position.y:.3f}."
        )
        future = self.navigateToPoseActionClient.send_goal_async(goal)
        future.add_done_callback(self.returnToInitialPoseGoalResponseCallback)
        self.currentFSMstate = Task3FSMState.WAIT_RETURN_TO_INITIAL_POSE

    def returnToInitialPoseGoalResponseCallback(self, future):
        goalHandle = future.result()  # TODO: add explicit exception handling.
        if not goalHandle.accepted:
            self.get_logger().error("[RETURN_TO_INITIAL_POSE] Goal rejected by Nav2.")
            self.returnToInitialPoseDone = True
            self.returnToInitialPoseSucceeded = False
            return

        self.get_logger().info("[RETURN_TO_INITIAL_POSE] Goal accepted by Nav2.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.returnToInitialPoseResultCallback)

    def returnToInitialPoseResultCallback(self, future):
        wrappedResult = future.result()  # TODO: add explicit exception handling.
        self.returnToInitialPoseDone = True
        self.returnToInitialPoseSucceeded = wrappedResult.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f"[RETURN_TO_INITIAL_POSE] Completed with status {wrappedResult.status}.")

    def handle_waitReturnToInitialPose(self):
        if not self.returnToInitialPoseDone:
            return

        if not self.returnToInitialPoseSucceeded:
            self.get_logger().error("[WAIT_RETURN_TO_INITIAL_POSE] Return to initial robot pose failed.")
            self.shouldShutdown = True
            return

        self.get_logger().info("[WAIT_RETURN_TO_INITIAL_POSE] Initial robot pose reached. Finishing Task3.")
        self.currentFSMstate = Task3FSMState.FINISH

    def handle_finish(self):
        self.get_logger().info("[FINISH] Task3 FSM completed.")
        self.shouldShutdown = True

def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = Task3FSMNode()
        while rclpy.ok() and not node.shouldShutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                f"KeyboardInterrupt received, shutting down {nodesParameters.task3FSMNodeName}..."
            )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
