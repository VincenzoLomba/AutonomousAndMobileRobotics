
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
from nav2_msgs.action import ComputePathToPose
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
from . import discoveryLoader
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
    effectiveGraspingTranslation: Optional[float] = None # Effective translation (in meters) that the Tiago torso will perform in order to reach the grasping/releasing pose from the approach pose (AND viceversa).
    @property
    def markerID(self) -> int: return self.markerInfo.markerID
    @property
    def markerNickname(self) -> str: return self.markerInfo.markerNickname

@dataclass
class ActionFlags:
    # A very simple class that will be used for storing the FLAGs related to a specific Action Server usage (e.g. TiagoArm or TiagoGripper)
    sent: bool = False
    done: bool = False
    succeeded: bool = False
    def reset(self):
        self.sent = False
        self.done = False
        self.succeeded = False

class Task3TransportationPhase(Enum):
    # During the Task3 execution, in the cube transportation process, a LOT of states will be re-used, with very very slight differences.
    # That said, in order to avoid a huge number of almost iddentical states, I have introduced this phase enumeration, that will be used to distinguish
    # the different phases of the cube transportation process, and to slightly change the behavior of the same states depending on the current phase.
    PICKING = 1
    PLACING = 2

class Task3CubeStatus(Enum):
    # Enumeration introduced for the same reason as the previous one, simply distinguishing grasped/ungrasped cube.
    UNGRASPED = 1
    GRASPED = 2

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
    NAVIGATE_TO_PLATFORM = 10
    CUBE_DETECTION = 12
    MOVE_ARM_TO_WORKING_CONFIGURATION = 14
    MOVE_GRIPPER_TO_CUBE_APPROACH = 16
    OPEN_GRIPPER = 18
    LOWER_TORSO_TO_PLATFORM = 21
    CLOSE_GRIPPER = 23
    MOVE_GRIPPER_BACK_TO_APPROACH_POSE = 25
    MOVE_ARM_TO_TRANSPORT_CONFIGURATION = 27
    MOVE_ARM_TO_HOME_CONFIGURATION = 41
    FINISH = 46

class ReorientaPhaseBeforeNav(Enum):
    COMPUTE_PATH = 0
    SPIN = 1
    NAVIGATE = 2
    ADJUST = 3
    FINSHED = 4
    FAILED = -1

class Task3FSMNode(Node):

    def __init__(self):

        super().__init__(params.task3FSMNodeName)
        self.get_logger().info(f"Starting {params.task3FSMNodeName} initialization...")

        # +---------------------------------------------------------------------------------------------------------------------------------------------+
        # | The following portion of code is exactly the same as the one used in the Task2 FSM Node for (1) arm tucking and (2) autonomous localization |
        # +---------------------------------------------------------------------------------------------------------------------------------------------+

        self.lastAMCLMsg = None
        self.latestJointState = None
        # self.collectArucoSamples = False
        # self.pickArucoSamples = []
        # self.placeArucoSamples = []
        # self.arucoLocationsSaved = False
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
        self.localizationGoodMetricConsecutiveCountRequired = params.localizationGoodMetricConsecutiveCountRequired
        self.localizationGoodMetricConsecutiveCount = 0
        self.robotXY = (0.0, 0.0)
        self.localizationSpinDone = False
        self.localizationSpinSucceeded = False
        self.localizationDriveOnHeadingDone = False
        self.localizationDriveOnHeadingSucceeded = False
        self.RANDOMWALKlocalization = False

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
        
        self.computePathFLAGs = ActionFlags()
        self.spinToPathHeadingFLAGs = ActionFlags()
        self.lastComputedPath = None # When in need of navigation to a platform, this variable stores the last computed global path
        # An enum/integer that indicates: -1 failed, 0 need computePathToGoal (before navigation), 1 need Spin (before navigation), 2 ready to navigate, 3 need Spin (after navigation), 4 navigation concluded
        self.navigationPhase = ReorientaPhaseBeforeNav.COMPUTE_PATH
        self.platformNavigationFLAGs = ActionFlags()
        self.lastComputedPlatformNavigationGoal = None # When in need of navigation to a platform, this variable stores the last computed navigation goal
        self.platformApproachReorientationFLAGs = ActionFlags()
        self.armWorkingConfigurationFLAGs = ActionFlags()
        self.gripperApproachPoseFLAGs = ActionFlags()
        self.openGripperFLAGs = ActionFlags()
        self.lowerTorsoToPlatformFLAGs = ActionFlags()
        self.closeGripperFLAGs = ActionFlags()
        self.returnToApproachPoseFLAGs = ActionFlags()
        self.moveArmToTransportConfigurationFLAGs = ActionFlags()
        self.moveArmToHomeConfigurationFLAGs = ActionFlags()

        self.headPanSweepIndex = 0 # During the cube detection phase, the Tiago head is panned to search for cubes on the table

        # During the transportation, the torso of the Tiago robot will be moved. The following variables will be involved in that motion.
        self.torsoMotionTarget = None # Last set target position for the torso motion.
        self.torsoMotionStartTime = None # Instant at which the torso motion was commanded (used to check for timeout).
        self.torsoMotionTimeout = 8.0 # Maximum time allowed for the single torso motion to complete (in seconds).
        self.torsoMotionTolerance = 0.005 # Tolerance for considering the single torso motion as completed (in meters).
        self.torsoMinSafePosition = 0.005 # Minimum safe position for the Tiago torso (in meters). The torso should never be commanded/lowered below this value.
        self.lastTorsoPositionInApproachPose = None # This variable will be used to store/retreive the last Tiago torso position

        self.initialRobotPose = None # In this variable, the whole POSE of the robot BEFORE starting the transportation will be stored

        self.transportationPhase = Task3TransportationPhase.PICKING # This variable will be used to distinguish between the PICKING and PLACING phases of the cube transportation process
        self.cubeStatus = Task3CubeStatus.UNGRASPED # This variable will be used to distinguish between the UNGRASPED and GRASPED status of the cube

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
        self.computePathActionClient = ActionClient(self, ComputePathToPose, params.computePathToPoseActionName)
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
        if self.currentCubeIndex < 0 or self.currentCubeIndex >= len(self.cubesData): return # All cubes have already benn processed (just a guard, this should never happen)
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
        elif self.currentFSMstate == Task3FSMState.TRIGGER_LOCALIZATION_RESTART: self.handle_triggerLocalizationRestart()
        elif self.currentFSMstate == Task3FSMState.WAIT_LOCALIZATION_RESTARTED: self.handle_waitLocalizationRestarted()
        elif self.currentFSMstate == Task3FSMState.EVALUATING_LOCALIZATION: self.handle_evaluatingLocalization()
        elif self.currentFSMstate == Task3FSMState.LOCALIZING: self.handle_localizing()
        # Cubes transportation between pick and place locations
        # Note that in the following FSM states I have used a DIFFERENT standard approach w.r.t previous ones:
        # I do NOT split anymore the single action in a couple of execute/wait states, but I handle that couple in an unique state.
        # In order to do that, I have added an additional FLAG for each state/action (the "sent" one)
        elif self.currentFSMstate == Task3FSMState.INIT_CUBES_TRANSPORTATION: self.handle_initCubesTransportation()
        # At this stage, we have "self.transportationPhase = Task3Phase.PICKING"
        elif self.currentFSMstate == Task3FSMState.NAVIGATE_TO_PLATFORM: self.handle_navigateToPlatform() # Navigating to the PICKING platform
        elif self.currentFSMstate == Task3FSMState.CUBE_DETECTION: self.handle_cubeDetection()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_WORKING_CONFIGURATION: self.handle_moveArmToWorkingConfiguration()
        elif self.currentFSMstate == Task3FSMState.MOVE_GRIPPER_TO_CUBE_APPROACH: self.handle_moveGripperToCubeApproach()
        elif self.currentFSMstate == Task3FSMState.OPEN_GRIPPER: self.handle_openGripper()
        elif self.currentFSMstate == Task3FSMState.LOWER_TORSO_TO_PLATFORM: self.handle_lowerTorsoToPlatform()
        elif self.currentFSMstate == Task3FSMState.CLOSE_GRIPPER: self.handle_closeGripper()
        elif self.currentFSMstate == Task3FSMState.MOVE_GRIPPER_BACK_TO_APPROACH_POSE: self.handle_moveGripperBackToApproachPose()
        # elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_WORKING_CONFIGURATION: self.handle_moveArmToWorkingConfiguration()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_TRANSPORT_CONFIGURATION: self.handle_moveArmToTransportConfiguration()
        # At this stage, we have "self.transportationPhase = Task3Phase.PLACING"
        # elif self.currentFSMstate == Task3FSMState.NAVIGATE_TO_PLATFORM: self.handle_navigateToPlatform() # Navigating to the PLACING platform
        # elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_WORKING_CONFIGURATION: self.handle_moveArmToWorkingConfiguration()
        # elif self.currentFSMstate == Task3FSMState.MOVE_GRIPPER_TO_CUBE_APPROACH: self.handle_moveGripperToCubeApproach()
        # elif self.currentFSMstate == Task3FSMState.LOWER_TORSO_TO_PLATFORM: self.handle_lowerTorsoToPlatform()
        # elif self.currentFSMstate == Task3FSMState.OPEN_GRIPPER: self.handle_openGripper()
        # elif self.currentFSMstate == Task3FSMState.MOVE_GRIPPER_BACK_TO_APPROACH_POSE: self.handle_moveGripperBackToApproachPose()
        # elif self.currentFSMstate == Task3FSMState.CLOSE_GRIPPER: self.handle_closeGripper()
        # elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_WORKING_CONFIGURATION: self.handle_moveArmToWorkingConfiguration()
        elif self.currentFSMstate == Task3FSMState.MOVE_ARM_TO_HOME_CONFIGURATION: self.handle_moveArmToHomeConfiguration()
        # At this stage, we have "self.transportationPhase = Task3Phase.PICKING"
        # elif self.currentFSMstate == Task3FSMState.NAVIGATE_TO_PLATFORM: self.handle_navigateToPlatform() # Navigating to the PICKING platform
        # repeat...
        elif self.currentFSMstate == Task3FSMState.FINISH: self.handle_finish()
        else: self.get_logger().error(f"Encountered an unknown FSM state: {self.currentFSMstate}")

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
                    # self.currentFSMstate = Task3FSMState.SET_HEAD_FOR_DISCOVERY
                    # In the Task3, after the autonomous localization, the robot has to start the transportation of the cubes between the pick and place locations
                    self.currentFSMstate = Task3FSMState.INIT_CUBES_TRANSPORTATION
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
    
    def extractYawFromQuaternion(self, orientationQuaternion):
        # A simple method that extracts the yaw angle from a quaternion
        siny_cosp = 2.0 * (orientationQuaternion.w * orientationQuaternion.z + orientationQuaternion.x * orientationQuaternion.y)
        cosy_cosp = 1.0 - 2.0 * (orientationQuaternion.y * orientationQuaternion.y + orientationQuaternion.z * orientationQuaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def getCurrentYawFromAMCLmessages(self):
        # A simple method that exploits the last available AMCL message to extract the current yaw of the robot in the map frame (from quaternion to euler angle)
        # Note that this method is used at a stage of the FSM at which at least one AMCL message has already been received (so self.lastAMCLMsg can be supposed not None)
        q = self.lastAMCLMsg.pose.pose.orientation
        return self.extractYawFromQuaternion(q)

    def normalizeAngle(self, angle):
        # A simple utiliy method to normalize an angle in radians to the range [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))
    
    # +----------------------------------------------------------------------------------------------------------------------------+
    # |                             End of the part fo the code related to Autonomous Localization                                 |
    # +----------------------------------------------------------------------------------------------------------------------------+

    # +------------------------------------------------------------------------------------------------------------------------+
    # |                      Transportation of all cubes from the Pick Location to the Place Location                          |
    # +------------------------------------------------------------------------------------------------------------------------+

    def computeShortSpin(self, currentYAW, targetYAW):
        # This methods takes as input a couple of angular positions (YAWs) and computes the shortest spin/rotation to get from the first to the second.
        return self.normalizeAngle(targetYAW - currentYAW)

    def publishTorsoliftCommand(self, targetPosition: float):
        # This method publishes a command to the torso lift joint to move it to the specified target position
        msg = JointTrajectory()
        msg.joint_names = [params.torsoLiftJointName]
        point = JointTrajectoryPoint()
        point.positions = [float(targetPosition)]
        point.time_from_start = Duration(sec = 2) # Imposed time duration for the torso movement
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
            self.torsoMotionTarget = None
            self.torsoMotionStartTime = None
            return True, True
        elapsedSeconds = (self.get_clock().now() - self.torsoMotionStartTime).nanoseconds * 1e-9
        if elapsedSeconds > self.torsoMotionTimeout:
            self.logErrorWithStatus(
                f"Torso motion timed out (timeout: {self.torsoMotionTimeout}s): current={float(currentTorsoPosition):.4f}, "
                f"target={float(self.torsoMotionTarget):.4f}, error={error:.4f}."
            )
            self.torsoMotionTarget = None
            self.torsoMotionStartTime = None
            return True, False
        self.logInfoWithStatus(
            f"Waiting torso motion complention: current={float(currentTorsoPosition):.4f}, "
            f"target={float(self.torsoMotionTarget):.4f}, error={error:.4f}."
        )
        return False, False

    def publishHeadTiltCommand(self, targetTilt):
        # This method can be used to publish a command to control the tilt-position of the Tiago head,
        # and it is used (in this Task3 FSM) to tilt the head correctly in order to detect the cubes on the pick location table.
        # This is literally the very same method used in the Task2 FSM Node
        current_head_pan = self.getJointPosition(params.panHeadJointName)
        if current_head_pan is None:
            self.logWarnWithStatus("Cannot read current " + params.panHeadJointName + " yet from topic " + params.jointStateTopic + ", retrying...")
            return False
        target_head_tilt = targetTilt
        msg = JointTrajectory()
        msg.joint_names = [params.panHeadJointName, params.tiltHeadJointName]
        point = JointTrajectoryPoint()
        point.positions = [current_head_pan, target_head_tilt]
        point.time_from_start = Duration(sec = 1) # Imposed time duration for the head movement
        msg.points.append(point)
        self.headCommandPublisher.publish(msg)
        self.logInfoWithStatus(f"Published head tilt command: {params.panHeadJointName}={current_head_pan:.3f}, {params.tiltHeadJointName}={target_head_tilt:.3f}.")
        return True
    
    def publishHeadPanCommand(self, targetPan):
        # This method can be used to publish a command to control the pan-position of the Tiago head,
        # and it is used (in this Task3 FSM) to pan the head correctly in order to detect the cubes on the pick location table.
        # This is literally the very same method used in the Task2 FSM Node
        current_head_tilt = self.getJointPosition(params.tiltHeadJointName)
        if current_head_tilt is None:
            self.logWarnWithStatus("Cannot read current " + params.tiltHeadJointName + " yet from topic " + params.jointStateTopic + ", retrying...")
            return False
        target_head_pan = targetPan
        msg = JointTrajectory()
        msg.joint_names = [params.panHeadJointName, params.tiltHeadJointName]
        point = JointTrajectoryPoint()
        point.positions = [target_head_pan, current_head_tilt]
        point.time_from_start = Duration(sec = params.headPanSweepPeriod) # Imposed time duration for the head movement
        msg.points.append(point)
        self.headCommandPublisher.publish(msg)
        self.logInfoWithStatus(f"Published head pan command: {params.panHeadJointName}={target_head_pan:.3f}, {params.tiltHeadJointName}={current_head_tilt:.3f}.")
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
        # (2) Tilt the Tiago head to a proper position in order to (later on) detect the cubes on the pick location table
        # (3) Load of the results of the cubes discovery (performed in Task2) from the related JSON files (indeed, the pick and place locations in the map)
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
            if self.publishHeadTiltCommand(params.headTiltDuringPickAndPlace): self.tiltHeadSent = True
        else:
            current_head_tilt = self.getJointPosition(params.tiltHeadJointName)
            if current_head_tilt is None:
                self.logInfoWithStatus("Waiting for a value for " + params.tiltHeadJointName + " from " + params.jointStateTopic + "...")
                return
            target_head_tilt = float(params.headTiltDuringPickAndPlace)
            head_tilt_error = abs(current_head_tilt - target_head_tilt)
            if head_tilt_error > params.headTiltTolerance:
                self.logInfoWithStatus(f"Waiting head tilt: current={current_head_tilt:.3f}, target={target_head_tilt:.3f}, error={head_tilt_error:.3f}.")
            else:
                self.logInfoWithStatus(f"Head tilt done: current={current_head_tilt:.3f}, target={target_head_tilt:.3f}, error={head_tilt_error:.3f}.")
                # (3) Load discovery results (the load is attempted only ONCE; TODO: implement a limited-in-amount retry mechanism in case of failure)
                self.logInfoWithStatus("Loading Task2 discovery results from JSON files...")
                try:
                    self.pickLocation, self.placeLocation = discoveryLoader.loadTask2LocationEstimates(self.savedMapPath)
                    # Note that the TF transforms broadcasting is not strictly required for the correct execution of the Task3 FSM,
                    # but it is useful for debugging and visualization purposes (e.g. in RViz) and therefore it has been performed.
                    self.broadcastTF(self.pickLocation.pose, self.pickLocation.frame_id, "task2_pick_location_estimate")
                    self.broadcastTF(self.placeLocation.pose, self.placeLocation.frame_id, "task2_place_location_estimate")
                    self.logInfoWithStatus("Task2 discovery results loaded successfully!")
                    self.transportationPhase = Task3TransportationPhase.PICKING
                    self.currentFSMstate = Task3FSMState.NAVIGATE_TO_PLATFORM
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

    def buildPlatformApproachGoal(self, platformLocation: params.ArucoPoseEstimate, approachDistance: float) -> NavigateToPose.Goal:
        # This method builds a navigation goal to approach a given platform (pick or place) at a specified distance from it.
        markerX = float(platformLocation.pose.position.x)
        markerY = float(platformLocation.pose.position.y)
        markerZ = float(platformLocation.pose.position.z)
        markerQX = float(platformLocation.pose.orientation.x)
        markerQY = float(platformLocation.pose.orientation.y)
        markerQZ = float(platformLocation.pose.orientation.z)
        markerQW = float(platformLocation.pose.orientation.w)

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
        self.broadcastTF(
            goal.pose.pose,
            parentFrameID=goal.pose.header.frame_id,
            frameID=f"task3_{platformLocation.label}_platform_approach",
        )
        self.lastComputedPlatformNavigationGoal = copy.deepcopy(goal)
        return goal

    def handle_navigateToPlatform(self):
        # In this state, the FSM sends a navigation goal to the Nav2 stack in order to approach the required platform.
        # The specific platform (pick OR place) is determined by the value of the variable self.transportationPhase
        # Note: at this stage, IF we are navigating to the pick platform, then the currently selected cube to be transported is
        # indeed ALREADY selected (and it's the one with index self.currentCubeIndex).
        # Three additional important notes about the final version of the code:
        # (1) when navigating back to the pick platform, an additional check is performed: IFF all cubes have already been processed,
        #     then the navigation goal is switched from the pick platform to the original robot pose.
        # (2) once the navigation towards a platform (pick or place) is completed, a final re-orientation (expliting Spin Action Server) towards the platform is performed
        # (3) in a newer/final version of the code, I have added a re-orientation phase before the navigation to the platform/origin.
        #     reorientationBeforeNavigationPhase is a FLAG that indicates if the robot should be re-oriented to the first pose of the computed path (if any),
        #     before starting the navigation to the platform.
        #     Values: -1 failed, 0 need computePathToGoal (before navigation), 1 need Spin (before navigation), 2 ready to navigate, 3 need Spin (after navigation), 4 navigation concluded
        navigateBackToInitialPose = not(self.currentCubeIndex < len(self.cubesData)) # A simple check to verify if all cubes have already been processed/transported
        navigationTarget = "initial_robot_pose" if navigateBackToInitialPose else f"{self.transportationPhase.name}_platform"
        if self.navigationPhase == ReorientaPhaseBeforeNav.COMPUTE_PATH:
            if not self.computePathFLAGs.sent:
                # Sending a request to the ComputePathToPose Action Server to compute a global path to the goal (the approach pose of the pick OR place platform)
                if not self.computePathActionClient.server_is_ready():
                    self.logInfoWithStatus("Waiting for ComputePathToPose action server...")
                    return
                # Building up the goal message for the ComputePathToPose Action Server
                goal = None
                if not navigateBackToInitialPose:
                    goal = self.buildPlatformApproachGoal(
                        self.pickLocation if self.transportationPhase == Task3TransportationPhase.PICKING else self.placeLocation,
                        params.platformApproachDistance
                    )
                else:
                    goal = NavigateToPose.Goal()
                    goal.pose.header.frame_id = self.initialRobotPose.header.frame_id
                    goal.pose.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.pose = self.initialRobotPose.pose
                goalMsg = ComputePathToPose.Goal()
                goalMsg.goal = goal.pose
                goalMsg.use_start = False # This FLAG indicated that the global path must be computed starting from the current robot pose
                # goalMsg.start = ... NOT used!
                self.computePathFLAGs.sent = True
                self.computePathFLAGs.done = False
                self.computePathFLAGs.succeeded = False
                self.lastComputedPath = None
                self.logInfoWithStatus(f"Now requesting path for the goal to the {navigationTarget}...")
                future = self.computePathActionClient.send_goal_async(goalMsg)
                future.add_done_callback(self.computePathGoalResponseCallback)
            else:
                # Waiting for the ComputePathToPose Action Server to complete the request
                if not self.computePathFLAGs.done: return
                if not self.computePathFLAGs.succeeded or self.lastComputedPath is None:
                    self.logWarnWithStatus(f"The ComputePathToPose request failed. Re-orientation is therefore skipped.")
                    self.navigationPhase = ReorientaPhaseBeforeNav.FAILED
                else: self.navigationPhase = ReorientaPhaseBeforeNav.SPIN
                self.computePathFLAGs.sent = False
        if self.navigationPhase == ReorientaPhaseBeforeNav.SPIN:
            # Sending a request to the Spin Action Server to re-orient the robot towards the heading of the computed path
            if not self.spinToPathHeadingFLAGs.sent:
                if not self.spinActionClient.server_is_ready():
                    self.logInfoWithStatus("Waiting for Spin Action Server...")
                    return
                if self.lastAMCLMsg is None:
                    self.logWarnWithStatus("Missing last AMCL pose, waiting for it to be available...")
                    return
                # Extracting the heading of the computed path (with the previously defined method)
                targetHeading = self.extractInitialPathHeading(self.lastComputedPath)
                if targetHeading is None:
                    self.logWarnWithStatus(f"It was not possibile to extract the path heading. Skipping the Spin Action and navigating towards the {navigationTarget}.")
                    self.navigationPhase = ReorientaPhaseBeforeNav.FAILED
                    return
                # Retreiving the current yaw of the robot and computing the shortest spin to reach the target heading (with the previously defined methods)
                currentYaw = self.getCurrentYawFromAMCLmessages()
                shortSpin = self.computeShortSpin(currentYaw, targetHeading)
                # Building up the goal message for the Spin Action Server
                goalMsg = Spin.Goal()
                goalMsg.target_yaw = float(shortSpin)
                goalMsg.time_allowance = Duration(sec = 30)
                self.spinToPathHeadingFLAGs.sent = True
                self.spinToPathHeadingFLAGs.done = False
                self.spinToPathHeadingFLAGs.succeeded = False
                self.logInfoWithStatus(f"Requiring a Spin to the Nav2 Stack: currentYaw = {currentYaw:.3f}, targetHeading={targetHeading:.3f}, shortSpin={shortSpin:.3f}")
                future = self.spinActionClient.send_goal_async(goalMsg)
                future.add_done_callback(self.makeSpinGoalResponseCallback(self.spinToPathHeadingFLAGs))
            else:
                if not self.spinToPathHeadingFLAGs.done: return
                if not self.spinToPathHeadingFLAGs.succeeded:
                    self.logWarnWithStatus(f"The Spin Action has been sent BUT has failed; navigating anyway towards the {navigationTarget}.")
                    self.navigationPhase = ReorientaPhaseBeforeNav.FAILED
                else: self.navigationPhase = ReorientaPhaseBeforeNav.NAVIGATE
                self.spinToPathHeadingFLAGs.sent = False
        if self.navigationPhase == ReorientaPhaseBeforeNav.NAVIGATE or self.navigationPhase == ReorientaPhaseBeforeNav.FAILED:
            # Handling the navigation to the pick OR place platform (depending on the value of self.transportationPhase),
            # OR to the initial robot pose (if all cubes have already been processed)
            if not self.platformNavigationFLAGs.sent:
                if not self.navigateToPoseActionClient.server_is_ready():
                    self.logInfoWithStatus("Waiting for NavigateToPose action server...")
                    return
                self.logInfoWithStatus(f"Sending to Nav2 a NavigateToPose goal to approach the {navigationTarget}.")
                goal = None
                if not navigateBackToInitialPose:
                    goal = self.buildPlatformApproachGoal(
                        self.pickLocation if self.transportationPhase == Task3TransportationPhase.PICKING else self.placeLocation,
                        params.platformApproachDistance
                    )
                else:
                    goal = NavigateToPose.Goal()
                    goal.pose.header.frame_id = self.initialRobotPose.header.frame_id
                    goal.pose.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.pose = self.initialRobotPose.pose
                self.platformNavigationFLAGs.sent = True
                self.platformNavigationFLAGs.done = False
                self.platformNavigationFLAGs.succeeded = False
                future = self.navigateToPoseActionClient.send_goal_async(goal)
                future.add_done_callback(self.platformNavigationGoalResponseCallback)
            else:
                if not self.platformNavigationFLAGs.done: return
                if not self.platformNavigationFLAGs.succeeded:
                    self.logErrorWithStatus(f"Navigation to {navigationTarget} failed, now retrying...")
                    # TODO: implement a WatchDog mechanism to avoid retrying indefinitely
                    self.platformNavigationFLAGs.reset()
                else:
                    self.logInfoWithStatus(f"Successfully navigated to the {navigationTarget}.")
                    self.platformNavigationFLAGs.sent = False
                    if navigateBackToInitialPose or not params.platformApproachReorientation:
                        self.navigationPhase = ReorientaPhaseBeforeNav.FINSHED
                    else:
                        self.navigationPhase = ReorientaPhaseBeforeNav.ADJUST
                    self.lastComputedPath = None
        if self.navigationPhase == ReorientaPhaseBeforeNav.ADJUST:
            if not self.platformApproachReorientationFLAGs.sent:
                # Sending a request to the Spin Action Server to re-orient the robot towards platform
                if self.lastComputedPlatformNavigationGoal is None:
                    self.logWarnWithStatus("Unexpectedly missing last platform navigation goal: skipping final re-orientation towards platform")
                    self.navigationPhase = ReorientaPhaseBeforeNav.FINSHED
                    return
                self.logInfoWithStatus(f"Successfully navigated to the {navigationTarget}. Now adjusting the robot pose to be exactly aligned with the platform...")
                # Extracting required in-place rotation (note that Spin Action Service and self.lastAMCLMsg are supposed to be available at this stage)
                currentYaw = self.getCurrentYawFromAMCLmessages()
                targetYaw = self.extractYawFromQuaternion(self.lastComputedPlatformNavigationGoal.pose.pose.orientation)
                spinYaw = self.computeShortSpin(currentYaw, targetYaw)
                # Building up the goal message for the Spin Action Server
                goalMsg = Spin.Goal()
                goalMsg.target_yaw = float(spinYaw)
                goalMsg.time_allowance = Duration(sec = 30)
                self.platformApproachReorientationFLAGs.sent = True
                self.platformApproachReorientationFLAGs.done = False
                self.platformApproachReorientationFLAGs.succeeded = False
                self.logInfoWithStatus(f"Requiring a Spin to the Nav2 Stack: currentYaw = {currentYaw:.3f}, targetYaw={targetYaw:.3f}, shortSpin={spinYaw:.3f}")
                future = self.spinActionClient.send_goal_async(goalMsg)
                future.add_done_callback(self.makeSpinGoalResponseCallback(self.platformApproachReorientationFLAGs))
            else:
                if not self.platformApproachReorientationFLAGs.done: return
                if not self.platformApproachReorientationFLAGs.succeeded:
                    self.logWarnWithStatus(f"The Spin Action has been sent BUT has failed; proceeding anyway to platform approach.")
                else:
                    self.logInfoWithStatus(f"Successfully re-oriented towards the {navigationTarget}.")
                self.platformApproachReorientationFLAGs.sent = False
                self.navigationPhase = ReorientaPhaseBeforeNav.FINSHED
        if self.navigationPhase == ReorientaPhaseBeforeNav.FINSHED:
            # Navigation concluded, now passing to the next state
            self.navigationPhase = ReorientaPhaseBeforeNav.COMPUTE_PATH
            # In case we have NOT navigated toward a platform, but we have navigated back to the initial robot pose, then we can directly conclude the navigation phase and pass to the next FSM state
            if navigateBackToInitialPose:
                self.logInfoWithStatus("Successfully navigated back to the initial robot pose.")
                self.currentFSMstate = Task3FSMState.FINISH
                return
            # Otherwise...
            if self.transportationPhase == Task3TransportationPhase.PICKING:
                # Once the picking location is reached, the next step is to detect the cube to be actually picked
                self.cubeDetectionWaitStartTime = None
                self.headPanSweepIndex = 0
                # self.cubesData[self.currentCubeIndex].arucoSamples.clear() # Not strictly necessary: just a guard
                self.currentFSMstate = Task3FSMState.CUBE_DETECTION
            else:
                # Once the placing location is reached, the next step is to move the arm to the working configuration,
                # so that then the gripper will be moved to the approach pose and that lowered to the platform (to place the grasped cube)
                self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_WORKING_CONFIGURATION

    def computePathGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if not goalHandle.accepted:
            self.logErrorWithStatus("ComputePathToPose goal unexpectedly rejected.")
            self.computePathFLAGs.done = True
            self.computePathFLAGs.succeeded = False
            return
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.computePathResultCallback)

    def computePathResultCallback(self, future):
        wrappedResult = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        self.computePathFLAGs.done = True
        self.computePathFLAGs.succeeded = (wrappedResult.status == GoalStatus.STATUS_SUCCEEDED)
        if self.computePathFLAGs.succeeded:
            self.lastComputedPath = wrappedResult.result.path
            self.logInfoWithStatus("ComputePathToPose received successfully!")
        else:
            self.lastComputedPath = None
            self.logErrorWithStatus(f"ComputePathToPose failed with status {wrappedResult.status}.")

    def makeSpinGoalResponseCallback(self, flags: ActionFlags):
        def spinResponseCallback(future):
            goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
            if not goalHandle.accepted:
                self.logErrorWithStatus("Spin goal unexpectedly rejected.")
                flags.done = True
                flags.succeeded = False
                return
            resultFuture = goalHandle.get_result_async()
            resultFuture.add_done_callback(self.makeSpinGoalResultCallback(flags))
        return spinResponseCallback

    def makeSpinGoalResultCallback(self, flags: ActionFlags):
        def spinResultCallback(future):
            wrappedResult = future.result()
            flags.done = True
            flags.succeeded = (wrappedResult.status == GoalStatus.STATUS_SUCCEEDED)
            self.logInfoWithStatus(f"Spin completed with status {wrappedResult.status}.")
        return spinResultCallback
    
    def platformNavigationGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: add explicit exception handling
        if not goalHandle.accepted:
            self.logErrorWithStatus(f"NavigateToPose goal rejected by Nav2.")
            self.platformNavigationFLAGs.done = True
            self.platformNavigationFLAGs.succeeded = False
            return
        self.logInfoWithStatus(f"NavigateToPose goal accepted by Nav2.")
        resultFuture = goalHandle.get_result_async()
        resultFuture.add_done_callback(self.platformNavigationResultCallback)

    def platformNavigationResultCallback(self, future):
        wrappedResult = future.result() # TODO: add explicit exception handling
        self.platformNavigationFLAGs.done = True
        self.platformNavigationFLAGs.succeeded = wrappedResult.status == GoalStatus.STATUS_SUCCEEDED
        self.logInfoWithStatus(f"NavigateToPose goal completed with status {wrappedResult.status}.")
        
    def handle_cubeDetection(self):
        # In this state, the FSM waits for a certain amount of time (defined by the parameter pickingWaitingTime) in order to collect
        # enough samples of the current cube marker before computing an estimate of its pose.
        # Very important: they are supposed to be true the following two conditions
        # (1) Tiago is in the joint-space-defined HOME configuration (params.HOME_JOINT_POSITIONS)
        # (2) The Tiago position and head-tilt are such that the cubes are in the camera view
        # If during this waiting period no samples are collected, then the collection is restarted from scratch (this is a very simple behavior).
        # Once the collection is completed, they are computed:
        # (1) the final pose estimate of the cube w.r.t. the map reference frame
        # (2) the final pose estimate of the cube w.r.t. the Tiago base_link reference frame
        # (3) the approach pose for the Tiago gripper to indeed approach the cube (again, w.r.t. the Tiago base_link reference frame), defined in the workspace.
        #     Note that this approach pose if exactly above the cube and is the one that the gripper should reach BEFORE moving to the cube itself.
        # Important note: in a newer/final version of the code, I have added a head pan sweep during the cube detection phase, in order to increase the chances of detecting the cube marker.
        if self.cubeDetectionWaitStartTime is None:
            # Starting collection
            self.collectCurrentCubeSamplesENABLER = True
            self.cubeDetectionWaitStartTime = self.get_clock().now()
            if params.headPanSweepEnabled: self.publishHeadPanCommand(params.headPanSweepPositions[self.headPanSweepIndex])
            self.logInfoWithStatus(f"Now collecting marker {self.cubesData[self.currentCubeIndex].markerID} samples for {params.pickingWaitingTime:.2f} seconds...")
        else:
            elapsedCollectionTime = (self.get_clock().now() - self.cubeDetectionWaitStartTime).nanoseconds * 1e-9
            timeRanOut = not(elapsedCollectionTime < float(params.pickingWaitingTime))
            # Verifying collection. 1: head pan progress
            if params.headPanSweepEnabled:
                currentHeadPan = self.getJointPosition(params.panHeadJointName)
                if currentHeadPan is None:
                    self.logInfoWithStatus("Waiting for a value for " + params.tiltHeadJointName + " from " + params.jointStateTopic + "...")
                    return
                if timeRanOut:
                    # Detection time ran out, now moving the head pan back to the default position
                    self.collectCurrentCubeSamplesENABLER = False
                    targetHeadPan = params.headPanDuringPickAndPlace
                    headPanError = abs(currentHeadPan - targetHeadPan)
                    if headPanError > params.headPanTolerance:
                        self.logInfoWithStatus(f"Passing to default head pan position: {params.headPanDuringPickAndPlace:.3f}.")
                        self.publishHeadPanCommand(params.headPanDuringPickAndPlace)
                        return
                else:
                    targetHeadPan = params.headPanSweepPositions[self.headPanSweepIndex]
                    headPanError = abs(currentHeadPan - targetHeadPan)
                    if headPanError > params.headPanTolerance:
                        self.logInfoWithStatus(f"Waiting head pan: current={currentHeadPan:.3f}, target={targetHeadPan:.3f}, error={headPanError:.3f}.")
                        return
                    self.logInfoWithStatus(f"Head pan done: current={currentHeadPan:.3f}, target={targetHeadPan:.3f}, error={headPanError:.3f}.")
                    self.headPanSweepIndex = (self.headPanSweepIndex + 1) % len(params.headPanSweepPositions)
                    self.logInfoWithStatus(f"Moving head pan to the next position in the sweep: {params.headPanSweepPositions[self.headPanSweepIndex]:.3f}.")
                    self.publishHeadPanCommand(params.headPanSweepPositions[self.headPanSweepIndex])
            # Verifying collection. 2: waiting time check
            if not timeRanOut: return
            self.collectCurrentCubeSamplesENABLER = False
            if len(self.cubesData[self.currentCubeIndex].arucoSamples) == 0:
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
            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.x = float(quaternion[0])
            pose.orientation.y = float(quaternion[1])
            pose.orientation.z = float(quaternion[2])
            pose.orientation.w = float(quaternion[3])
            self.cubesData[self.currentCubeIndex].finalPoseMap = params.ArucoPoseEstimate(
                found = True, # Whether at least one valid marker sample was collected.
                marker_id = self.cubesData[self.currentCubeIndex].markerID, # Numerical ID of the ArUco marker associated with this location.
                label = self.cubesData[self.currentCubeIndex].markerNickname, # Semantic role of the location, e.g. "pick" or "place".
                frame_id = frameId, # Reference frame in which the estimated pose is expressed, typically "map" ("map" is indeed used as a default value).
                marker_frame = self.cubesData[self.currentCubeIndex].markerInfo.markerFrame, # TF frame name associated with the specific ArUco marker.
                sample_count = len(samples), # Number of samples used to compute the estimate.
                pose = pose # The estimated pose of the ArUco marker in the specified frame_id.
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
                f"{len(samples)} samples and successfully stored (BOTH w.r.t. map/base_link reference frames)."
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
                [cubePose.orientation.x, cubePose.orientation.y, cubePose.orientation.z, cubePose.orientation.w],
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
                f"position={approachPose.position}, quat_xyzw={approachPose.orientation}."
            )
            self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_WORKING_CONFIGURATION

    # +--------------------------------------------------------------------------------------------------------------------------+
    # | A NOVELTY ELEMENT: the TiagoArm Action Server and the Grippe rAction Server require very similar triplet of callbacks    |
    # | (feedback, goal response, result), and therefore the code is structured in a very similar way for both of them. They are |
    # | are also re-used a lot of times. That said, I have defined an unique parametrizable triplet of callbacks for them!       |
    # +--------------------------------------------------------------------------------------------------------------------------+

    def makeActionFeedbackCallback(self):
        def feedbackCallback(feedbackMsg):
            self.logInfoWithStatus(f"Feedback: {feedbackMsg.feedback.current_state}")
        return feedbackCallback

    def makeGoalResponseCallback(self, flags: ActionFlags, actionServerName: str, resultCallback):
        def goalResponseCallback(future):
            goalHandle = future.result() # TODO: add explicit exception handling.
            if not goalHandle.accepted:
                self.logErrorWithStatus(f"Goal rejected by the {actionServerName} Action Server.")
                flags.done = True
                flags.succeeded = False
                return
            self.logInfoWithStatus(f"Goal accepted by the {actionServerName} Action Server.")
            resultFuture = goalHandle.get_result_async()
            resultFuture.add_done_callback(resultCallback)
        return goalResponseCallback
    
    def makeResultCallback(self, flags: ActionFlags, actionServerName: str):
        def resultCallback(future):
            result = future.result().result  # TODO: add explicit exception handling.
            if result.success:
                self.logInfoWithStatus(f"{actionServerName} success: {result.message}")
                flags.done = True
                flags.succeeded = True
            else:
                self.logErrorWithStatus(f"{actionServerName} failed: {result.message}")
                flags.done = True
                flags.succeeded = False
        return resultCallback
    
    # +---------------------------------------------------------------------------------------------------------------+
    # | End of the definition of the parametrizable triplet of callbacks for the TiagoArm and Gripper Action Servers. |
    # +---------------------------------------------------------------------------------------------------------------+

    def handle_moveArmToWorkingConfiguration(self):
        # In this state, the FSM exploits the TiagoArm Action Server in order to move the arm to a predefined working configuration (joint-space defined).
        # Note that this working configuration is used in-between the home/transportation configurations and the approach pose for the gripper to pick/place cubes.
        # Remember that: the home configiration is also defined in the joint space, meanwhile the approach pose is defined in the workspace.
        # To summarize:
        # Home configuration: uniquely defined in the joint space
        # Working configuration: uniquely defined in the joint space
        # Approach pose: redefined for each cube in the workspace (i.e. cartesian space)
        # For instance, when we are in the PICKING phase:
        # (1) The Tiago arrives to the pick location, and moves the gripper from the home configuration to the approach pose PASSING THROUGH the working configuration.
        # (2) Then, AFTER having picked the cube, the Tiago moves from the approach pose to the transportation configuration PASSING THROUGH the working configuration.
        # Analogously, when we are in the PLACING phase:
        # (1) The Tiago arrives to the place location, and moves the gripper from the transportation configuration to the approach pose PASSING THROUGH the working configuration.
        # (2) Then, AFTER having placed the cube, the Tiago moves from the approach pose to the home configuration PASSING THROUGH the working configuration.
        #
        # A very important remark about the working configuration:
        # Once the gripper will be moved to the approach pose, the Tiago torso will be lowered to make the gripper reach the cube (and grasp/release it).
        # That is, it will be supposed that the Tiago torso is ALREADY extended in order to be effectively lowered!
        # For that reason, the working configuration SHOULD ABSOLUTELY also be related to an HIGH Tiago torso extension!
        actionServerName = "TiagoArm"
        if not self.tiagoArmActionClient.server_is_ready():
            self.logInfoWithStatus(f"Waiting for {actionServerName} Action Server...")
            return
        if not self.armWorkingConfigurationFLAGs.sent:
            # Defining the goal message to be sent to the TiagoArm Action Server
            goalMsg = TiagoArm.Goal()
            goalMsg.joint_positions = params.PICKING_JOINT_POSITIONS
            goalMsg.position_obj = []
            goalMsg.quat_xyzw_obj = []
            self.armWorkingConfigurationFLAGs.sent = True
            self.armWorkingConfigurationFLAGs.done = False
            self.armWorkingConfigurationFLAGs.succeeded = False
            self.logInfoWithStatus(f"Sending working joint configuration to {actionServerName} Action Server.")
            future = self.tiagoArmActionClient.send_goal_async(goalMsg, feedback_callback = self.makeActionFeedbackCallback())
            future.add_done_callback(self.makeGoalResponseCallback(
                self.armWorkingConfigurationFLAGs,
                actionServerName,
                self.makeResultCallback(self.armWorkingConfigurationFLAGs, actionServerName)
            ))
        else:
            if not self.armWorkingConfigurationFLAGs.done: return
            if not self.armWorkingConfigurationFLAGs.succeeded: # TODO: implement some WatchDog mechanism to avoid retrying indefinitely
                self.logErrorWithStatus("Arm to working configuration motion failed, now retrying...")
                self.armWorkingConfigurationFLAGs.reset()
                return
            self.armWorkingConfigurationFLAGs.sent = False
            self.logInfoWithStatus("Arm reached working configuration.")
            if self.transportationPhase == Task3TransportationPhase.PICKING:
                # We are in the PICKING phase
                if self.cubeStatus == Task3CubeStatus.UNGRASPED:
                    # Once the arm has reached the working configuration, the next step is to move the gripper to the approach pose for the current cube
                    self.currentFSMstate = Task3FSMState.MOVE_GRIPPER_TO_CUBE_APPROACH
                else:
                    # Once the arm has reached the working configuration, the next step is to move back to the transportation configuration
                    self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_TRANSPORT_CONFIGURATION
            else:
                # We are in the PLACING phase
                if self.cubeStatus == Task3CubeStatus.GRASPED:
                    # Once the arm has reached the working configuration, the next step is to move the gripper to the approach pose for the current cube
                    self.currentFSMstate = Task3FSMState.MOVE_GRIPPER_TO_CUBE_APPROACH
                else:
                    # Once the arm has reached the working configuration, the next step is to move back to the home configuration
                    self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_HOME_CONFIGURATION

    def handle_moveGripperToCubeApproach(self):
        # In this state, the FSM exploits the TiagoArm Action Server in order to move the gripper to the approach pose for the current cube,
        # which has been previously computed and that is a pose in the Cartesian space.
        # Note that, at this stage, the Tiago Arm is ALWAYS supposed to be in the working configuration (joint space),
        # and the Tiago torso is ALREADY extended in order to be effectively lowered!
        # Very important note regarding the next state:
        # (1) If we are in the PICKING phase, then the next state will be to open the gripper in order to prepare for grasping the cube.
        # (2) If we are in the PLACING phase, then the next state will be to move down the Tiago torso in order to place the cube.
        actionServerName = "TiagoArm"
        if not self.tiagoArmActionClient.server_is_ready():
            self.logInfoWithStatus(f"Waiting for {actionServerName} Action Server...")
            return
        if not self.gripperApproachPoseFLAGs.sent:
            # Defining the goal message to be sent to the TiagoArm Action Server
            approachPose = copy.deepcopy(self.cubesData[self.currentCubeIndex].approachPose)
            if self.transportationPhase == Task3TransportationPhase.PLACING:
                approachPose.position.x += float(params.placingApproachForwardOffset)
            goalMsg = TiagoArm.Goal()
            goalMsg.joint_positions = []
            goalMsg.position_obj = [approachPose.position.x, approachPose.position.y, approachPose.position.z]
            goalMsg.quat_xyzw_obj = [approachPose.orientation.x, approachPose.orientation.y, approachPose.orientation.z, approachPose.orientation.w]
            self.gripperApproachPoseFLAGs.sent = True
            self.gripperApproachPoseFLAGs.done = False
            self.gripperApproachPoseFLAGs.succeeded = False
            self.logInfoWithStatus(f"Sending gripper approach pose to {actionServerName} Action Server.")
            future = self.tiagoArmActionClient.send_goal_async(goalMsg, feedback_callback = self.makeActionFeedbackCallback())
            future.add_done_callback(self.makeGoalResponseCallback(
                self.gripperApproachPoseFLAGs,
                actionServerName,
                self.makeResultCallback(self.gripperApproachPoseFLAGs, actionServerName)
            ))
        else:
            if not self.gripperApproachPoseFLAGs.done: return
            if not self.gripperApproachPoseFLAGs.succeeded: # TODO: implement some WatchDog mechanism to avoid retrying indefinitely
                self.logErrorWithStatus("Gripper to approach pose motion failed, now retrying...")
                self.gripperApproachPoseFLAGs.reset()
                return
            self.gripperApproachPoseFLAGs.sent = False
            self.logInfoWithStatus("Gripper reached cube approach pose.")
            if self.transportationPhase == Task3TransportationPhase.PICKING:
                # Once the gripper has reached the approach pose, the next step is to open the gripper in order to prepare for grasping the cube.
                self.currentFSMstate = Task3FSMState.OPEN_GRIPPER
            else:
                # Once the gripper has reached the approach pose, the next step is to move down the Tiago torso in order to then place the cube.
                self.currentFSMstate = Task3FSMState.LOWER_TORSO_TO_PLATFORM

    def handle_openGripper(self):
        # In this state, the FSM exploits the TiagoGripper Action Server in order to open the gripper.
        # Two situations are possible:
        # (1) If we are in the PICKING phase, then the gripper is opened in order to prepare for grasping the cube.
        # (2) If we are in the PLACING phase, then the gripper is opened in order to release the cube.
        actionServerName = "TiagoGripper"
        if not self.tiagoGripperActionClient.server_is_ready():
            self.logInfoWithStatus(f"Waiting for {actionServerName} Action Server...")
            return
        if not self.openGripperFLAGs.sent:
            # Defining the goal message to be sent to the TiagoGripper Action Server
            goalMsg = TiagoGripper.Goal()
            goalMsg.open = True
            goalMsg.object_model_name = ""
            goalMsg.object_link_name = ""
            self.openGripperFLAGs.sent = True
            self.openGripperFLAGs.done = False
            self.openGripperFLAGs.succeeded = False
            self.logInfoWithStatus(f"Sending open gripper goal to {actionServerName} Action Server.")
            future = self.tiagoGripperActionClient.send_goal_async(goalMsg, feedback_callback = self.makeActionFeedbackCallback())
            future.add_done_callback(self.makeGoalResponseCallback(
                self.openGripperFLAGs,
                actionServerName,
                self.makeResultCallback(self.openGripperFLAGs, actionServerName)
            ))
        else:
            if not self.openGripperFLAGs.done: return
            if not self.openGripperFLAGs.succeeded: # TODO: implement some WatchDog mechanism to avoid retrying indefinitely
                self.logErrorWithStatus("Gripper opening failed, now retrying...")
                self.openGripperFLAGs.reset()
                return
            self.openGripperFLAGs.sent = False
            self.logInfoWithStatus("Gripper opened successfully.")
            if self.transportationPhase == Task3TransportationPhase.PICKING:
                # Once the gripper has been opened, the next step is to move down the Tiago torso in order to have the gripper moving to the grasping pose.
                self.currentFSMstate = Task3FSMState.LOWER_TORSO_TO_PLATFORM
            else:
                # Once the gripper has been opened (and the cube released),
                # the next step is to move up the Tiago torso in order to have the gripper moving back to the approach pose.
                self.cubeStatus = Task3CubeStatus.UNGRASPED
                self.currentFSMstate = Task3FSMState.MOVE_GRIPPER_BACK_TO_APPROACH_POSE

    def handle_lowerTorsoToPlatform(self):
        # In this state, the FSM exploits the Tiago Torso Publisher (to the proper topic) to move down the Tiago torso in order to have the Tiago gripper
        # moving just above the pick/place platform, AKA to the pose at which the cube will be grasped OR released.
        # Note that at this stage the gripper has already reached the approach pose, that has been defined to have the gripper exactly above the cube expected location.
        currentTorsoPosition = self.getJointPosition(params.torsoLiftJointName)
        if currentTorsoPosition is None:
            self.logInfoWithStatus("Waiting for a value for " + params.torsoLiftJointName + " from " + params.jointStateTopic + "...")
            return
        if not self.lowerTorsoToPlatformFLAGs.sent:
            # Building up the torso target to be published
            self.lastTorsoPositionInApproachPose = currentTorsoPosition # Storing the actual torso position (the one corresponding to the actual approach pose)
            requestedDescent = None
            effectiveDescent = None
            if self.transportationPhase == Task3TransportationPhase.PICKING:
                # Retrieving the requested descent (that has been defined in reltation to params.cubeApproachHeight)
                requestedDescent = float(params.verticalTranslationGrasping)
                # Computing the available descent
                availableDescent = max(0.0, float(currentTorsoPosition) - float(self.torsoMinSafePosition)) # Important: it has been experimentally verified that the Tiago torso can be safely lowered by at least 0.05 meters (5 centimeters)
                # Computing the effective descent (the one that will be actually performed)
                effectiveDescent = min(requestedDescent, availableDescent)
                # Storing the final effective descent
                self.cubesData[self.currentCubeIndex].effectiveGraspingTranslation = float(effectiveDescent)
            else:
                # Retrieving the exact descent that has been used when previously grasping the cube AND reducing it by a safety delta (to avoid collisions with the platform when placing the cube)
                requestedDescent = max(0.0, self.cubesData[self.currentCubeIndex].effectiveGraspingTranslation - params.safetyPlacingDeltaTranslation)
                # Computing the available descent
                availableDescent = max(0.0, float(currentTorsoPosition) - float(self.torsoMinSafePosition))
                # Computing the effective descent (the one that will be actually performed)
                effectiveDescent = min(requestedDescent, availableDescent)            
            if effectiveDescent <= 1e-4: # 0.1mm
                self.logErrorWithStatus(
                    f"It seems that the Tiago torso is already at its minimum safe position, and therefore the gripper cannot be lowered to the grasping/placing pose. "
                    f"Check the parameters file and change in a proper manner the values that define the approach and grasping/placing poses AND in particular the working configuration. "
                    f"Shutting down the Task3 FSM. "
                    f"current_torso={float(currentTorsoPosition):.4f}, "
                    f"torso_min_safe={float(self.torsoMinSafePosition):.4f}."
                )
                self.currentFSMstate = Task3FSMState.FINISH
                return
            if effectiveDescent < requestedDescent:
                self.logWarnWithStatus(
                    f"ATTENTION: the requested descent {requestedDescent:.4f} exceeds the available torso descent {availableDescent:.4f}. "
                    f"Using reduced descent {effectiveDescent:.4f} for marker {self.cubesData[self.currentCubeIndex].markerID} (hoping to reach a useful grasping/placing pose). "
                    f"This should NOT happen. "
                    f"Check the parameters file and change in a proper manner the values that define the approach and grasping/placing poses AND in particular the working configuration."
                )
            targetTorsoPosition = float(currentTorsoPosition) - float(effectiveDescent)
            self.lowerTorsoToPlatformFLAGs.sent = True
            self.lowerTorsoToPlatformFLAGs.done = False
            self.lowerTorsoToPlatformFLAGs.succeeded = False
            self.logInfoWithStatus(f"Lowering the Tiago torso to reach the gripper grasping/placing pose")
            self.publishTorsoliftCommand(targetTorsoPosition)
        else:
            # Wait for the torso motion to be completed
            self.lowerTorsoToPlatformFLAGs.done, self.lowerTorsoToPlatformFLAGs.succeeded = self.isTorsoMotionCompleted()
            if not self.lowerTorsoToPlatformFLAGs.done: return
            if not self.lowerTorsoToPlatformFLAGs.succeeded: # TODO: implement some WatchDog mechanism to avoid retrying indefinitely
                self.logErrorWithStatus("Torso descent to grasping/placing pose failed, now retrying...")
                self.lowerTorsoToPlatformFLAGs.reset()
                return
            self.lowerTorsoToPlatformFLAGs.sent = False
            self.logInfoWithStatus(f"Torso descent to grasping/placing pose completed successfully for cube with ID {self.cubesData[self.currentCubeIndex].markerID}.")
            if self.transportationPhase == Task3TransportationPhase.PICKING:
                # Once the torso has been lowered to the grasping pose, the next step is to close the gripper in order to grasp the cube.
                self.currentFSMstate = Task3FSMState.CLOSE_GRIPPER
            else:
                # Once the torso has been lowered to the placing pose, the next step is to open the gripper in order to release the cube.
                self.currentFSMstate = Task3FSMState.OPEN_GRIPPER

    def handle_closeGripper(self):
        # In this state, the FSM exploits the TiagoGripper Action Server in order to close the gripper.
        # Two situations are possible:
        # (1) If we are in the PICKING phase, then the gripper is closed in order to grasp the cube.
        # (2) If we are in the PLACING phase, then the gripper is closed in order to then navigate back the Tiago with a closed gripper.
        #     This closure is performed just after having lifted up the torso and just before moving back to the working configuration.
        actionServerName = "TiagoGripper"
        if not self.tiagoGripperActionClient.server_is_ready():
            self.logInfoWithStatus(f"Waiting for {actionServerName} Action Server...")
            return
        if not self.closeGripperFLAGs.sent:
            # Defining the goal message to be sent to the TiagoGripper Action Server
            goalMsg = TiagoGripper.Goal()
            goalMsg.open = False
            # Note that we are effectively performing a grasp ONLY IF we are in the PICKING phase.
            goalMsg.object_model_name = self.cubesData[self.currentCubeIndex].markerInfo.gazeboModelName if self.transportationPhase == Task3TransportationPhase.PICKING else ""
            goalMsg.object_link_name = self.cubesData[self.currentCubeIndex].markerInfo.gazeboModelLinkName if self.transportationPhase == Task3TransportationPhase.PICKING else ""
            self.closeGripperFLAGs.sent = True
            self.closeGripperFLAGs.done = False
            self.closeGripperFLAGs.succeeded = False
            self.logInfoWithStatus(f"Sending close gripper goal to {actionServerName} Action Server.")
            future = self.tiagoGripperActionClient.send_goal_async(goalMsg, feedback_callback = self.makeActionFeedbackCallback())
            future.add_done_callback(self.makeGoalResponseCallback(
                self.closeGripperFLAGs,
                actionServerName,
                self.makeResultCallback(self.closeGripperFLAGs, actionServerName)
            ))
        else:   
            if not self.closeGripperFLAGs.done: return
            if not self.closeGripperFLAGs.succeeded: # TODO: implement some WatchDog mechanism to avoid retrying indefinitely
                self.logErrorWithStatus("Gripper closing failed, now retrying...")
                self.closeGripperFLAGs.reset()
                return
            self.closeGripperFLAGs.sent = False
            self.logInfoWithStatus("Gripper closed successfully.")
            if self.transportationPhase == Task3TransportationPhase.PICKING:
                # Once the gripper has been closed (and the cube grasped), the next step is to move back the Tiago torso to the original approach pose.
                self.cubeStatus = Task3CubeStatus.GRASPED
                self.currentFSMstate = Task3FSMState.MOVE_GRIPPER_BACK_TO_APPROACH_POSE
            else:
                # Once the empty gripper has been closed, the next step is to move back to the working configuration.
                self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_WORKING_CONFIGURATION

    def handle_moveGripperBackToApproachPose(self):
        # In this state, after a successful grasping/release of the cube, the FSM moves back the Tiago torso AKA the gripper is moved to the original approach pose.
        # After this motion, two situations are possible:
        # (1) If we are in the PICKING phase, then the gripper is grasping a cube and the next step is gonna be to move back to the working configuration.
        # (2) If we are in the PLACING phase, then the gripper has just released a cube and the next step is gonna to be closing the empty gripper before moving back to the working configuration.
        if not self.returnToApproachPoseFLAGs.sent:
            # Building up the torso target to be published
            self.returnToApproachPoseFLAGs.sent = True
            self.returnToApproachPoseFLAGs.done = False
            self.returnToApproachPoseFLAGs.succeeded = False
            self.publishTorsoliftCommand(self.lastTorsoPositionInApproachPose)
            self.logInfoWithStatus("Raising the Tiago torso back to the original approach pose.")
        else:
            # Wait for the torso motion to be completed
            self.returnToApproachPoseFLAGs.done, self.returnToApproachPoseFLAGs.succeeded = self.isTorsoMotionCompleted()
            if not self.returnToApproachPoseFLAGs.done: return
            if not self.returnToApproachPoseFLAGs.succeeded: # TODO: implement some WatchDog mechanism to avoid retrying indefinitely
                self.logErrorWithStatus("Torso lift back to approach pose failed, now retrying...")
                self.returnToApproachPoseFLAGs.reset()
                return
            self.returnToApproachPoseFLAGs.sent = False
            self.logInfoWithStatus("Torso lift back to approach pose completed successfully.")
            if self.transportationPhase == Task3TransportationPhase.PICKING:
                # Once the torso has been lifted back to the approach pose, the next step is to move back to the working configuration.
                self.currentFSMstate = Task3FSMState.MOVE_ARM_TO_WORKING_CONFIGURATION
            else:
                # Once the torso has been lifted back to the approach pose, the next step is to close the gripper in order to prepare for navigating back with a closed gripper.
                self.currentFSMstate = Task3FSMState.CLOSE_GRIPPER

    def handle_moveArmToTransportConfiguration(self):
        # In this state, the FSM exploits the TiagoArm Action Server in order to move the arm to a predefined transport configuration (joint positions).
        # Very important: at this stage, AFTER having moved to the transport configuration, the Tiago robot will start navigating.
        # That is, at this stage we expect to be in a PICKING phase, AND the self.transportationPhase variable will be modified from PICKING to PLACING.
        actionServerName = "TiagoArm"
        if not self.tiagoArmActionClient.server_is_ready():
            self.logInfoWithStatus(f"Waiting for {actionServerName} Action Server...")
            return
        if not self.moveArmToTransportConfigurationFLAGs.sent:
            # Defining the goal message to be sent to the TiagoArm Action Server
            goalMsg = TiagoArm.Goal()
            goalMsg.joint_positions = params.TRANSPORTATION_JOINT_POSITIONS
            goalMsg.position_obj = []
            goalMsg.quat_xyzw_obj = []
            self.moveArmToTransportConfigurationFLAGs.sent = True
            self.moveArmToTransportConfigurationFLAGs.done = False
            self.moveArmToTransportConfigurationFLAGs.succeeded = False
            self.logInfoWithStatus(f"Sending transport joint configuration to {actionServerName} Action Server.")
            future = self.tiagoArmActionClient.send_goal_async(goalMsg, feedback_callback = self.makeActionFeedbackCallback())
            future.add_done_callback(self.makeGoalResponseCallback(
                self.moveArmToTransportConfigurationFLAGs,
                actionServerName,
                self.makeResultCallback(self.moveArmToTransportConfigurationFLAGs, actionServerName)
            ))
        else:
            if not self.moveArmToTransportConfigurationFLAGs.done: return
            if not self.moveArmToTransportConfigurationFLAGs.succeeded: # TODO: implement some WatchDog mechanism to avoid retrying indefinitely
                self.logErrorWithStatus("Arm to transport configuration motion failed, now retrying...")
                self.moveArmToTransportConfigurationFLAGs.reset()
                return
            self.moveArmToTransportConfigurationFLAGs.sent = False
            self.logInfoWithStatus("Arm reached transport configuration.")
            # Updating the transportation phase (from PICKING to PLACING)
            self.transportationPhase = Task3TransportationPhase.PLACING
            # Once the arm has reached the transport configuration, the next step is to navigate to the place platform.
            self.currentFSMstate = Task3FSMState.NAVIGATE_TO_PLATFORM
    
    def handle_moveArmToHomeConfiguration(self):
        # In this state, the FSM exploits the TiagoArm Action Server in order to move the arm to the predefined home configuration (joint positions),
        # before navigating back to the pick platform to grasp the next cube OR navigating back to the initial robot pose if all cubes have been already processed.
        # Important: at this stage, we expect to be in a PLACING phase, and we starting navigating back to picking platform / initial pose, that is,
        # the self.transportationPhase variable will be modified from PLACING to PICKING.
        actionServerName = "TiagoArm"
        if not self.tiagoArmActionClient.server_is_ready():
            self.logInfoWithStatus(f"Waiting for {actionServerName} Action Server...")
            return
        if not self.moveArmToHomeConfigurationFLAGs.sent:
            # Defining the goal message to be sent to the TiagoArm Action Server
            goalMsg = TiagoArm.Goal()
            goalMsg.joint_positions = params.HOME_JOINT_POSITIONS
            goalMsg.position_obj = []
            goalMsg.quat_xyzw_obj = []
            self.moveArmToHomeConfigurationFLAGs.sent = True
            self.moveArmToHomeConfigurationFLAGs.done = False
            self.moveArmToHomeConfigurationFLAGs.succeeded = False
            self.logInfoWithStatus(f"Sending home joint configuration to {actionServerName} Action Server.")
            future = self.tiagoArmActionClient.send_goal_async(goalMsg, feedback_callback = self.makeActionFeedbackCallback())
            future.add_done_callback(self.makeGoalResponseCallback(
                self.moveArmToHomeConfigurationFLAGs,
                actionServerName,
                self.makeResultCallback(self.moveArmToHomeConfigurationFLAGs, actionServerName)
            ))
        else:
            if not self.moveArmToHomeConfigurationFLAGs.done: return
            if not self.moveArmToHomeConfigurationFLAGs.succeeded: # TODO: implement some WatchDog mechanism to avoid retrying indefinitely
                self.logErrorWithStatus("Arm to home configuration motion failed, now retrying...")
                self.moveArmToHomeConfigurationFLAGs.reset()
                return
            self.moveArmToHomeConfigurationFLAGs.sent = False
            self.logInfoWithStatus("Arm reached home configuration.")
            # Updating the transportation phase (from PLACING to PICKING)
            self.transportationPhase = Task3TransportationPhase.PICKING
            # Once the arm has reached the home configuration, the next step is to navigate back to the pick platform OR to the initial robot pose.
            self.currentCubeIndex += 1
            if self.currentCubeIndex < len(self.cubesData):
                # There are still cubes to be moved, so we navigate back to the pick platform.
                
                self.logInfoWithStatus(f"Moving to next cube marker {self.cubesData[self.currentCubeIndex].markerID}. Returning to pick platform navigation.")
                self.currentFSMstate = Task3FSMState.NAVIGATE_TO_PLATFORM
            else:
                # All cubes have been moved, so we navigate back to the initial robot pose.
                self.logInfoWithStatus("All cubes have been transported. Returning to initial robot pose.")
                self.currentFSMstate = Task3FSMState.NAVIGATE_TO_PLATFORM
                # self.currentFSMstate = Task3FSMState.RETURN_TO_INITIAL_POSE
                # In the final version of the code, the final return to the initial robot pose is delegated to the "NAVIGATE_TO_PLATFORM" state.

    def handle_finish(self):
        self.logInfoWithStatus("Task3 FSM is now shutting down.")
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
                f"KeyboardInterrupt received, shutting down {params.task3FSMNodeName}..."
            )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__": main()
