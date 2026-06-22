
# This Python files defines a ROS2 Humble Node which implements the FSM that implements the logic that executes the Task1

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from tiago_exam_interfaces.action import TiagoArm
from explore_lite_msgs.msg import ExploreStatus
from enum import Enum
from . import tiagoExamParameters as params
from nav2_msgs.srv import SaveMap
from rclpy.parameter import Parameter
import os
from rcl_interfaces.srv import SetParameters
import math

class Task1FSMState(Enum):
    TUCK_ARM = 1
    WAIT_ARM_TUCKED = 2
    DISABLE_YAW_GOAL_CHECKER = 9
    WAIT_DISABLED_YAW_GOAL_CHECKER = 10
    START_EXPLORE_LITE = 3
    WAIT_EXPLORATION_START = 4
    WAIT_EXPLORATION_COMPLETE = 5
    SAVE_MAP = 6
    WAIT_SAVE_MAP = 7
    SHUTDOWN = 8

class Task1FSMNode(Node):

    def __init__(self):

        super().__init__(params.task1FSMNodeName)
        self.get_logger().info(f"Starting {params.task1FSMNodeName} initialization...")

        self.exploreLiteExploreResumeTopicLabel = params.exploreLiteExploreResumeTopicLabel
        self.exploreLiteStatusTopicLabel = params.exploreLiteStatusTopicLabel
        self.exploreLiteNodeName = params.exploreLiteNodeName
        self.exploreLiteStatus = None
        self.yawGoalToleranceParamName = params.yawToleranceGoalCheckerParamName
        
        self.armTuckGoalDone = False
        self.armTuckGoalSucceeded = False
        self.mapSaveDone = False
        self.mapSaveSucceeded = False
        self.shouldShutdown = False
        self.disableYAWGoalCheckerDone = False
        self.disableYAWGoalCheckerSucceeded = False

        # Create the publisher for the explore/resume topic to be able to start/pause the autonomous exploration managed by the ExploreLite Node
        self.exploreLiteExploreResumeTopicPublisher = self.create_publisher(Bool, self.exploreLiteExploreResumeTopicLabel, 10) # remark: 10 is the length of this publisher queue, i.e. the maximum number of messages that can be buffered before being sent out (if the subscriber is not receiving & processing them fast enough)
        # Subscribe to the explore/status topic to get the current status of the ExploreLite Node (i.e. understand the state of the autonomous exploration)
        self.exploreLiteStatusSubscription = self.create_subscription(ExploreStatus, self.exploreLiteStatusTopicLabel, self.exploreLiteStatusCallback, 10) # remark: same as above
        # Initialize the Action Client for the TiagoArm Action Server (exposed by the TiagoArm Node) to be able to send goal for the arm motion
        self.tiagoArmActionClient = ActionClient(self, TiagoArm, params.tiagoArmActionName) # Action Client
        # Initialize the Service Client for the SaveMap Service (exposed by the Nav2 Map Saver Node) to be able to send requests for saving the final generated map
        self.saveMapClient = self.create_client(SaveMap, params.nav2SaveMapServiceName) # Service Client
        # Initialize the Service Client for setting parameters on the controller server (used to online get and set the yaw_goal_tolerance value of the Goal Checker used within the Nav2 Stack)
        self.controllerServerSetParametersClient = self.create_client(SetParameters, params.controllerServerSetParametersServiceName)

        # Initialize the internal state
        self.currentFSMstate = Task1FSMState.TUCK_ARM

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
        self.get_logger().info(f"Task1 FSM node initialized. Initial state: {self.currentFSMstate.name}.")

    def exploreLiteStatusCallback(self, msg): self.exploreLiteStatus = msg.status

    def stepUpFSM(self):
        if self.currentFSMstate == Task1FSMState.TUCK_ARM: self.handle_tuckArm()
        elif self.currentFSMstate == Task1FSMState.WAIT_ARM_TUCKED: self.handle_waitArmTucked()
        elif self.currentFSMstate == Task1FSMState.DISABLE_YAW_GOAL_CHECKER: self.handle_disableYawGoalChecker()
        elif self.currentFSMstate == Task1FSMState.WAIT_DISABLED_YAW_GOAL_CHECKER: self.handle_waitDisabledYawGoalChecker()
        elif self.currentFSMstate == Task1FSMState.START_EXPLORE_LITE: self.handle_startExploreLite()
        elif self.currentFSMstate == Task1FSMState.WAIT_EXPLORATION_START: self.handle_waitExplorationStart()
        elif self.currentFSMstate == Task1FSMState.WAIT_EXPLORATION_COMPLETE: self.handle_waitExplorationComplete()
        elif self.currentFSMstate == Task1FSMState.SAVE_MAP: self.handle_saveMap()
        elif self.currentFSMstate == Task1FSMState.WAIT_SAVE_MAP: self.handle_waitSaveMap()
        elif self.currentFSMstate == Task1FSMState.SHUTDOWN: self.handle_shutdown()
        else: self.get_logger().error(f"Encountered an unknown FSM state: {self.currentFSMstate}")

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
        self.currentFSMstate = Task1FSMState.WAIT_ARM_TUCKED

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
                self.currentFSMstate = Task1FSMState.DISABLE_YAW_GOAL_CHECKER
            else:
                self.logErrorWithStatus("Arm goal failed or was rejected by TiagoArm Action Server. Retrying...")
                self.currentFSMstate = Task1FSMState.TUCK_ARM

    def disableYawGoalCheckerCallback(self, future):
        try:
            response = future.result()
            if len(response.results) != 1:
                self.disableYAWGoalCheckerDone = True
                self.disableYAWGoalCheckerSucceeded = False
                self.logErrorWithStatus(
                    f"Unexpected empty SetParameters response for parameter '{self.yawGoalToleranceParamName}': the parameter may have remained unchanged."
                )
                return
            # Retrieve the result of the SetParameters request
            result = response.results[0]
            self.disableYAWGoalCheckerDone = True
            self.disableYAWGoalCheckerSucceeded = result.successful
            if result.successful: self.logInfoWithStatus(f"{self.yawGoalToleranceParamName} successfully set to 2pi rad.")
            else:
                self.logErrorWithStatus(
                    f"Failed to set '{self.yawGoalToleranceParamName}' to 360 deg / "
                    f"2pi rad. Reason from {params.controllerServerSetParametersServiceName}: {result.reason}."
                )
        except Exception as e:
            self.disableYAWGoalCheckerDone = True
            self.disableYAWGoalCheckerSucceeded = False
            self.logErrorWithStatus(f"Exception while setting '{self.yawGoalToleranceParamName}' to 360 deg / 2pi rad (the parameter may have remained unchanged): {e}.")

    def handle_disableYawGoalChecker(self):
        if not self.controllerServerSetParametersClient.service_is_ready():
            self.logInfoWithStatus(f"Waiting for {params.controllerServerSetParametersServiceName} service...")
            return
        # Building up a SetParameters request
        req = SetParameters.Request()
        param = Parameter(self.yawGoalToleranceParamName, Parameter.Type.DOUBLE, 2.0 * math.pi)
        req.parameters = [param.to_parameter_msg()]
        self.disableYAWGoalCheckerDone = False
        self.disableYAWGoalCheckerSucceeded = False
        future = self.controllerServerSetParametersClient.call_async(req)
        future.add_done_callback(self.disableYawGoalCheckerCallback)
        self.returnToInitYawToleranceRequestSent = True
        self.logInfoWithStatus(f"Requested parameters '{params.yawToleranceGoalCheckerParamName}' to be set equal to 2pi rad.")
        self.currentFSMstate = Task1FSMState.WAIT_DISABLED_YAW_GOAL_CHECKER

    def handle_waitDisabledYawGoalChecker(self):
        # Note that the outcome of the SetParameters request is handled AND LOGGED in the disableYawGoalCheckerCallback() method
        # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
        if self.disableYAWGoalCheckerDone: self.currentFSMstate = Task1FSMState.START_EXPLORE_LITE

    def isExploreLiteReady(self) -> bool:
        exploreResumeTopicSubscriptionsInfo = self.get_subscriptions_info_by_topic(self.exploreLiteExploreResumeTopicLabel)
        if not exploreResumeTopicSubscriptionsInfo: return False
        amount = len(exploreResumeTopicSubscriptionsInfo)
        for endpoint in exploreResumeTopicSubscriptionsInfo:
            self.logInfoWithStatus(
                f"Found subscriber (out of {amount}) on 'explore/resume' -> "
                f"name: {endpoint.node_name}, "
                f"namespace: {endpoint.node_namespace}, "
                f"type: {endpoint.topic_type}"
            )
            if endpoint.node_name == self.exploreLiteNodeName: return True
        return False

    def handle_startExploreLite(self):
        if not self.isExploreLiteReady():
            self.logInfoWithStatus(f"ExploreLite subscriber '{self.exploreLiteNodeName}' not detected yet, still waiting for it...")
            # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
            return
        self.logInfoWithStatus(f"ExploreLite detected as ready: subscriber '{self.exploreLiteNodeName}' detected on '{self.exploreLiteExploreResumeTopicLabel}'.")
        msg = Bool()
        msg.data = True
        self.exploreLiteExploreResumeTopicPublisher.publish(msg)
        self.logInfoWithStatus(f"Published True on 'explore/resume', now waiting for ExploreLite to actually starting exploration...")
        self.currentFSMstate = Task1FSMState.WAIT_EXPLORATION_START

    def handle_waitExplorationStart(self):
        if self.exploreLiteStatus == ExploreStatus.EXPLORATION_STARTED or self.exploreLiteStatus == ExploreStatus.EXPLORATION_IN_PROGRESS:
            self.logInfoWithStatus(f"ExploreLite reported status '{self.exploreLiteStatus}', thus assuming exploration has started and passing to wait for its completion...")
            self.currentFSMstate = Task1FSMState.WAIT_EXPLORATION_COMPLETE
        else:
            self.logInfoWithStatus(f"Waiting for ExploreLite to start exploration... Current ExploreLite status: {self.exploreLiteStatus}")
            # TODO: implement some WatchDog mechanism to avoid waiting indefinitely

    def handle_waitExplorationComplete(self):
        # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
        # In a previous version of the code, the YAW Goal Checker was disabled only when returning to the origin;
        # in the final version, it's disabled from the very beginning of the exploration.
        # if self.exploreLiteStatus == ExploreStatus.RETURNING_TO_ORIGIN:
        #     # The Tiago robot is returning to origin: we only care for reaching the origin point in terms of location and NOT orientation
        #     if not self.returnToInitYawToleranceRequestSent: self.requestReturnToInitYawToleranceDisable()
        #     return
        if self.exploreLiteStatus == ExploreStatus.EXPLORATION_COMPLETE:
            self.logInfoWithStatus("ExploreLite reported a completed exploration, proceeding to map saving.")
            self.currentFSMstate = Task1FSMState.SAVE_MAP
        elif self.exploreLiteStatus == ExploreStatus.RETURN_TO_ORIGIN_FAILED:
            self.logInfoWithStatus("ExploreLite reported a completed exploration followed by a return to origin failed, proceeding to map saving anyway.")
            self.currentFSMstate = Task1FSMState.SAVE_MAP
        else:
            # Avoid an excessive amount of log messages during the exploration!
            # self.get_logger().info(f"[WAIT_EXPLORATION_COMPLETE] Waiting for exploration to be completed... Current ExploreLite status: {self.exploreLiteStatus}")
            pass

    def handle_saveMap(self):
        if not self.saveMapClient.service_is_ready(): # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
            self.logInfoWithStatus(f"Before map saving, waiting for {params.nav2SaveMapServiceName} Service...")
            return
        os.makedirs(self.savedMapPath, exist_ok = True)
        mapURL = os.path.join(self.savedMapPath, self.savedMapName)
        self.logInfoWithStatus(f"Saving map at '{mapURL}'...")
        req = SaveMap.Request()
        req.map_topic = params.nav2MapTopic
        req.map_url = mapURL
        req.image_format = "pgm"
        req.map_mode = "trinary" # cells are divided in free/empty/unknown
        req.free_thresh = 0.25 # standard value as indicated by the doc
        req.occupied_thresh = 0.65 # standard value as indicated by the doc
        self.mapSaveDone = False
        self.mapSaveSucceeded = False
        saveMapFuture = self.saveMapClient.call_async(req)
        saveMapFuture.add_done_callback(self.saveMapCallback)
        self.currentFSMstate = Task1FSMState.WAIT_SAVE_MAP

    def saveMapCallback(self, future):
        response = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        self.mapSaveDone = True
        if response.result: self.mapSaveSucceeded = True
        else:
            self.logErrorWithStatus("SaveMap Service failed and returned False as a result.")
            self.mapSaveSucceeded = False
    
    def handle_waitSaveMap(self):
        self.logInfoWithStatus("Waiting for map saving to be completed...")
        if self.mapSaveDone:
            if self.mapSaveSucceeded:
                self.logInfoWithStatus("Map saving completed successfully!")
                self.currentFSMstate = Task1FSMState.SHUTDOWN
            else:
                self.logWarnWithStatus("Map saving failed. Retrying...")
                self.currentFSMstate = Task1FSMState.SAVE_MAP
    
    def handle_shutdown(self):
        self.logInfoWithStatus("Task1 FSM Node is shutting down...")
        self.FSMtimer.cancel()
        self.shouldShutdown = True

    def logInfoWithStatus(self, message): self.get_logger().info(f"[{self.currentFSMstate.name}] {message}")
    def logWarnWithStatus(self, message): self.get_logger().warn(f"[{self.currentFSMstate.name}] {message}")
    def logErrorWithStatus(self, message): self.get_logger().error(f"[{self.currentFSMstate.name}] {message}")

def main(args = None):
    node = None
    try:
        rclpy.init(args = args)
        node = Task1FSMNode()
        while rclpy.ok() and not node.shouldShutdown: rclpy.spin_once(node, timeout_sec = 0.1) # spin_once() spins (IF present, otherwise it waits once the indicated time) the single first available callback, then immediatly returns)
    except KeyboardInterrupt:
        if node is not None: node.get_logger().info(f"KeyboardInterrupt received, shutting down {params.task1FSMNodeName}...")
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__": main()
