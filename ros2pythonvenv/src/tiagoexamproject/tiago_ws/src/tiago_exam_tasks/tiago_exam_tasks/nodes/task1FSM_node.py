
# This Python files defines a ROS2 Humble Node which implements the FSM that implements the logic that executes the Task1

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from tiago_exam_interfaces.action import TiagoArm
from explore_lite_msgs.msg import ExploreStatus
from enum import Enum
from . import nodesParameters
from nav2_msgs.srv import SaveMap
from rclpy.parameter import Parameter
import os

class Task1FSMState(Enum):
    TUCK_ARM = 1
    WAIT_ARM_TUCKED = 2
    START_EXPLORE_LITE = 3
    WAIT_EXPLORATION_START = 4
    WAIT_EXPLORATION_COMPLETE = 5
    SAVE_MAP = 6
    WAIT_SAVE_MAP = 7
    FINAL = 8

class Task1FSMNode(Node):

    def __init__(self):

        super().__init__(nodesParameters.task1FSMNodeName)
        self.get_logger().info(f"Starting {nodesParameters.task1FSMNodeName} initialization...")
        self.exploreLiteExploreResumeTopicLabel = nodesParameters.exploreLiteExploreResumeTopicLabel
        self.exploreLiteStatusTopicLabel = nodesParameters.exploreLiteStatusTopicLabel
        self.exploreLiteNodeName = nodesParameters.exploreLiteNodeName
        self.armTuckGoalDone = False
        self.armTuckGoalSucceeded = False
        self.exploreLiteStatus = None
        self.mapSaveDone = False
        self.mapSaveSucceeded = False
        self.shouldShutdown = False

        # Create the publisher for the explore/resume topic to be able to start/pause the autonomous exploration managed by the ExploreLite Node
        self.exploreLiteExploreResumeTopicPublisher = self.create_publisher(Bool, self.exploreLiteExploreResumeTopicLabel, 10) # remark: 10 is the length of this publisher queue, i.e. the maximum number of messages that can be buffered before being sent out (if the subscriber is not receiving & processing them fast enough)
        # Subscribe to the explore/status topic to get the current status of the ExploreLite Node (i.e. understand the state of the autonomous exploration)
        self.exploreLiteStatusSubscription = self.create_subscription(ExploreStatus, self.exploreLiteStatusTopicLabel, self.exploreLiteStatusCallback, 10) # remark: same as above
        # Initialize the Action Client for the TiagoArm Action Server (exposed by the TiagoArm Node) to be able to send goal for the arm motion
        self.tiagoArmActionClient = ActionClient(self, TiagoArm, nodesParameters.tiagoArmActionName) # Action Client
        # Initialize the Service Client for the SaveMap Service (exposed by the Nav2 Map Saver Node) to be able to send requests for saving the final generated map
        self.saveMapClient = self.create_client(SaveMap, nodesParameters.nav2SaveMapServiceName) # Service Client

        self.currentFSMstate = Task1FSMState.TUCK_ARM

        mapSavePathDefaultValue = nodesParameters.mapSavePathParameterDefaultValue
        self.declare_parameter(nodesParameters.mapSavePathParameterName, Parameter.Type.STRING)
        self.savedMapPath = self.get_parameter(nodesParameters.mapSavePathParameterName).value
        if self.savedMapPath is None or str(self.savedMapPath).strip() == "":
            self.get_logger().warn(f"None, empty or invalid {nodesParameters.mapSavePathParameterName} parameter, falling back to default value '{mapSavePathDefaultValue}'.")
            self.savedMapPath = mapSavePathDefaultValue
        else: self.savedMapPath = str(self.savedMapPath).strip()

        mapSaveNameDefaultValue = nodesParameters.mapSaveNameParameterDefaultValue
        self.declare_parameter(nodesParameters.mapSaveNameParameterName, Parameter.Type.STRING)
        self.savedMapName = self.get_parameter(nodesParameters.mapSaveNameParameterName).value
        if self.savedMapName is None or str(self.savedMapName).strip() == "":
            self.get_logger().warn(f"None or empty {nodesParameters.mapSaveNameParameterName} parameter, falling back to default value '{mapSaveNameDefaultValue}'.")
            self.savedMapName = mapSaveNameDefaultValue
        else: self.savedMapName = str(self.savedMapName).strip()

        fsmTimerPeriodDefaultValue = nodesParameters.fsmTimerPeriodParameterDefaultValue
        self.declare_parameter(nodesParameters.fsmTimerPeriodParameterName, Parameter.Type.DOUBLE)
        fsmTimerPeriod = self.get_parameter(nodesParameters.fsmTimerPeriodParameterName).value
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
        elif self.currentFSMstate == Task1FSMState.START_EXPLORE_LITE: self.handle_startExploreLite()
        elif self.currentFSMstate == Task1FSMState.WAIT_EXPLORATION_START: self.handle_waitExplorationStart()
        elif self.currentFSMstate == Task1FSMState.WAIT_EXPLORATION_COMPLETE: self.handle_waitExplorationComplete()
        elif self.currentFSMstate == Task1FSMState.SAVE_MAP: self.handle_saveMap()
        elif self.currentFSMstate == Task1FSMState.WAIT_SAVE_MAP: self.handle_waitSaveMap()
        elif self.currentFSMstate == Task1FSMState.FINAL: self.handle_final()
        else: self.get_logger().error(f"Encountered an unknown FSM state: {self.currentFSMstate}")

    def handle_tuckArm(self):
        if not self.tiagoArmActionClient.server_is_ready():
            self.get_logger().info("[TUCK_ARM] Waiting for TiagoArm Action Server...")
            return
        self.get_logger().info("[TUCK_ARM] Sending goal to TiagoArm Action Server...")
        goalMsg = TiagoArm.Goal()
        goalMsg.joint_positions = nodesParameters.HOME_JOINT_POSITIONS
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
        self.get_logger().info(f"[TUCK_ARM] Feedback: {feedback.current_state}")

    def armGoalResponseCallback(self, future):
        goalHandle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if not goalHandle.accepted:
            self.get_logger().error("[TUCK_ARM] Goal rejected by TiagoArm Action Server.")
            self.armTuckGoalDone = True
            self.armTuckGoalSucceeded = False
            return
        self.get_logger().info("[TUCK_ARM] Goal accepted by TiagoArm Action Server.")
        getResultFuture = goalHandle.get_result_async()
        getResultFuture.add_done_callback(self.armResultCallback)

    def armResultCallback(self, future):
        result = future.result().result # TODO: insert this line within a try-except block and manage possible exceptions in a proper way
        if result.success:
            self.get_logger().info(f"[TUCK_ARM] Success: {result.message}")
            self.armTuckGoalDone = True
            self.armTuckGoalSucceeded = True
        else:
            self.get_logger().error(f"[TUCK_ARM] Failed: {result.message}")
            self.armTuckGoalDone = True
            self.armTuckGoalSucceeded = False

    def handle_waitArmTucked(self):
        self.get_logger().info("[WAIT_ARM_TUCKED] Waiting for arm to be tucked...")
        # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
        if self.armTuckGoalDone:
            if self.armTuckGoalSucceeded:
                self.get_logger().info("[WAIT_ARM_TUCKED] Arm motion completed.")
                self.currentFSMstate = Task1FSMState.START_EXPLORE_LITE
            else:
                self.get_logger().warn("[WAIT_ARM_TUCKED] Arm goal failed or was rejected by TiagoArm Action Server. Retrying...")
                self.currentFSMstate = Task1FSMState.TUCK_ARM

    def isExploreLiteReady(self) -> bool:
        exploreResumeTopicSubscriptionsInfo = self.get_subscriptions_info_by_topic(self.exploreLiteExploreResumeTopicLabel)
        if not exploreResumeTopicSubscriptionsInfo: return False
        amount = len(exploreResumeTopicSubscriptionsInfo)
        for endpoint in exploreResumeTopicSubscriptionsInfo:
            self.get_logger().info(
                f"[WAIT_EXPLORE_LITE] Found subscriber (out of {amount}) on 'explore/resume' -> "
                f"name: {endpoint.node_name}, "
                f"namespace: {endpoint.node_namespace}, "
                f"type: {endpoint.topic_type}"
            )
            if endpoint.node_name == self.exploreLiteNodeName: return True
        return False

    def handle_startExploreLite(self):
        if not self.isExploreLiteReady():
            self.get_logger().info(f"[START_EXPLORE_LITE] ExploreLite subscriber '{self.exploreLiteNodeName}' not detected yet, still waiting for it...")
            # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
            return
        self.get_logger().info(f"[START_EXPLORE_LITE] ExploreLite detected as ready: subscriber '{self.exploreLiteNodeName}' detected on '{self.exploreLiteExploreResumeTopicLabel}'.")
        msg = Bool()
        msg.data = True
        self.exploreLiteExploreResumeTopicPublisher.publish(msg)
        self.get_logger().info("[START_EXPLORE_LITE] Published True on 'explore/resume', now waiting for ExploreLite to actually starting exploration...")
        self.currentFSMstate = Task1FSMState.WAIT_EXPLORATION_START

    def handle_waitExplorationStart(self):
        if self.exploreLiteStatus == ExploreStatus.EXPLORATION_STARTED or self.exploreLiteStatus == ExploreStatus.EXPLORATION_IN_PROGRESS:
            self.get_logger().info(f"[WAIT_EXPLORATION_START] ExploreLite reported status '{self.exploreLiteStatus}', thus assuming exploration has started and passing to wait for its completion...")
            self.currentFSMstate = Task1FSMState.WAIT_EXPLORATION_COMPLETE
        else:
            self.get_logger().info(f"[WAIT_EXPLORATION_START] Waiting for ExploreLite to start exploration... Current ExploreLite status: {self.exploreLiteStatus}")
            # TODO: implement some WatchDog mechanism to avoid waiting indefinitely

    def handle_waitExplorationComplete(self):
        # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
        if self.exploreLiteStatus == ExploreStatus.EXPLORATION_COMPLETE or self.exploreLiteStatus == ExploreStatus.RETURNING_TO_ORIGIN:
            self.get_logger().info("[WAIT_EXPLORATION_COMPLETE] ExploreLite reported a completed exploration, proceeding to map saving.")
            self.currentFSMstate = Task1FSMState.SAVE_MAP
        elif self.exploreLiteStatus == ExploreStatus.RETURN_TO_ORIGIN_FAILED:
            self.get_logger().warn("[WAIT_EXPLORATION_COMPLETE] ExploreLite reported a completed exploration followed by a return to origin failed, proceeding to map saving anyway.")
            self.currentFSMstate = Task1FSMState.SAVE_MAP
        else:
            # Avoid an excessive amount of log messages during the exploration!
            # self.get_logger().info(f"[WAIT_EXPLORATION_COMPLETE] Waiting for exploration to be completed... Current ExploreLite status: {self.exploreLiteStatus}")
            pass

    def handle_saveMap(self):
        if not self.saveMapClient.service_is_ready(): # TODO: implement some WatchDog mechanism to avoid waiting indefinitely
            self.get_logger().info(f"[SAVE_MAP] Before map saving, waiting for {nodesParameters.nav2SaveMapServiceName} Service...")
            return
        os.makedirs(self.savedMapPath, exist_ok = True)
        mapURL = os.path.join(self.savedMapPath, self.savedMapName)
        self.get_logger().info(f"[SAVE_MAP] Saving map at '{mapURL}'...")
        req = SaveMap.Request()
        req.map_topic = nodesParameters.nav2MapTopic
        req.map_url = mapURL
        req.image_format = "pgm"
        req.map_mode = "trinary" # cells are divided in free/empty/unknown
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
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
            self.get_logger().error("[SAVE_MAP] SaveMap Service failed and returned False as a result.")
            self.mapSaveSucceeded = False
    
    def handle_waitSaveMap(self):
        self.get_logger().info("[WAIT_SAVE_MAP] Waiting for map saving to be completed...")
        if self.mapSaveDone:
            if self.mapSaveSucceeded:
                self.get_logger().info("[WAIT_SAVE_MAP] Map saving completed successfully!")
                self.currentFSMstate = Task1FSMState.FINAL
            else:
                self.get_logger().warn("[WAIT_SAVE_MAP] Map saving failed. Retrying...")
                self.currentFSMstate = Task1FSMState.SAVE_MAP
    
    def handle_final(self):
        self.get_logger().info("[FINAL] Task1 FSM Node completed all its steps successfully! Now shutting down the node...")
        self.FSMtimer.cancel()
        self.shouldShutdown = True

def main(args = None):
    node = None
    try:
        rclpy.init(args = args)
        node = Task1FSMNode()
        while rclpy.ok() and not node.shouldShutdown: rclpy.spin_once(node, timeout_sec = 0.1) # spin_once() spins (IF present, otherwise it waits once the indicated time) the single first available callback, then immediatly returns 
    except KeyboardInterrupt:
        if node is not None: node.get_logger().info(f"KeyboardInterrupt received, shutting down {nodesParameters.task1FSMNodeName}...")
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__": main()
