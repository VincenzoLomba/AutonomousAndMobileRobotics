
# This Python files defines a ROS2 Humble Node which implements the FSM that implements the logic that executes the Task1

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from tiago_exam_interfaces.action import TiagoArm
from enum import Enum
from . import nodesParameters

class Task1FSMState(Enum):
    WAIT_FOR_EXPLORE_LITE = 1
    PAUSE_EXPLORE_LITE = 2
    TUCK_ARM = 3
    WAIT_ARM_TUCKED = 4
    RESUME_EXPLORE_LITE = 5
    WAIT_EXPLORATION_COMPLETE = 6

class Task1FSMNode(Node):

    def __init__(self):

        super().__init__("task1_fsm_node")
        self.get_logger().info("Starting task1_fsm_node initialization...")
        self.exploreLiteExploreResumeTopicLabel = "explore/resume"
        self.exploreLiteNodeEndpointname = "explore_node"
        self.armGoalSent = False
        self.armGoalDone = False

        self.exploreLiteExploreResumeTopicPublisher = self.create_publisher(Bool, self.exploreLiteExploreResumeTopicLabel, 10) # remark: 10 is the length of this publisher queue, i.e. the maximum number of messages that can be buffered before being sent out (if the subscriber is not receiving & processing them fast enough)
        self.tiagoArmActionClient = ActionClient(self, TiagoArm, nodesParameters.tiagoArmActionName)

        self.currentState = Task1FSMState.WAIT_FOR_EXPLORE_LITE
        self.fsmTimer = self.create_timer(0.5, self.stepUpFSM) # A timer created like that (alias with self.create_timer) is gonna be spun as soon as this node is spun via rclpy.spin()
        self.get_logger().info(f"Task1 FSM node initialized. Initial state: {self.currentState.name}.")

    def stepUpFSM(self):
        if self.currentState == Task1FSMState.WAIT_FOR_EXPLORE_LITE: self.handle_waitForExploreLite()
        elif self.currentState == Task1FSMState.PAUSE_EXPLORE_LITE: self.handle_pauseExploreLite()
        elif self.currentState == Task1FSMState.TUCK_ARM: self.handle_tuckArm()
        elif self.currentState == Task1FSMState.WAIT_ARM_TUCKED: self.handle_waitArmTucked()
        elif self.currentState == Task1FSMState.RESUME_EXPLORE_LITE: self.handle_resumeExploreLite()
        elif self.currentState == Task1FSMState.WAIT_EXPLORATION_COMPLETE: self.handle_waitExplorationComplete()
        else: self.get_logger().error(f"Encountered an unknown FSM state: {self.currentState}")

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
            if endpoint.node_name == self.exploreLiteNodeEndpointname: return True
        return False

    def handle_waitForExploreLite(self):
        if self.isExploreLiteReady():
            self.get_logger().info(f"ExploreLite detected as ready: subscriber '{self.exploreLiteNodeEndpointname}' detected on '{self.exploreLiteExploreResumeTopicLabel}'.")
            self.currentState = Task1FSMState.PAUSE_EXPLORE_LITE
        else:
            self.get_logger().info(f"[WAIT_EXPLORE_LITE] ExploreLite subscriber '{self.exploreLiteNodeEndpointname}' not detected yet, still waiting for it...")

    def handle_pauseExploreLite(self):
        msg = Bool()
        msg.data = False
        self.exploreLiteExploreResumeTopicPublisher.publish(msg)
        self.get_logger().info("[PAUSE_EXPLORE_LITE] Published False on 'explore/resume'. ExploreLite should pause (and will be assumed to be paused).")
        self.currentState = Task1FSMState.TUCK_ARM

    def handle_tuckArm(self):
        if not self.tiagoArmActionClient.wait_for_server(timeout_sec = 0.5): # Waiting for the Action Server exposed by the TiagoArm Node to be online...
            self.get_logger().info("[TUCK_ARM] Waiting for TiagoArm action server...")
            return
        if not self.armGoalSent:
            self.get_logger().info("[TUCK_ARM] Sending goal to TiagoArm action server...")
            goalMsg = TiagoArm.Goal()
            goalMsg.joint_positions = nodesParameters.HOME_JOINT_POSITIONS
            self.sendGoalFuture = self.tiagoArmActionClient.send_goal_async(
                goalMsg,
                feedback_callback = self.armFeedbackCallback
            )
            self.sendGoalFuture.add_done_callback(self.armGoalResponseCallback)
            self.armGoalSent = True
            self.currentState = Task1FSMState.WAIT_ARM_TUCKED

    def armGoalResponseCallback(self, future):
        goal_handle = future.result() # TODO: insert this line within a try-except block and manage possible exceptions in some way
        if not goal_handle.accepted:
            self.get_logger().error("[TUCK_ARM] Goal rejected by TiagoArm action server.") # TODO: manage goal rejection in some way, e.g. retry sending the goal after some time, or abort the whole FSM and set it in an error state, etc...
            self.armGoalDone = True
            return
        self.get_logger().info("[TUCK_ARM] Goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.armResultCallback)

    def armFeedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[TUCK_ARM] Feedback: {feedback.current_state}")

    def armResultCallback(self, future):
        result = future.result().result
        if result.success: self.get_logger().info(f"[TUCK_ARM] Success: {result.message}")
        else:
            # TODO: manage failure is some way
            self.get_logger().error(f"[TUCK_ARM] Failed: {result.message}")
        self.armGoalDone = True

    def handle_waitArmTucked(self):
        if self.armGoalDone:
            self.get_logger().info("[WAIT_ARM_TUCKED] Arm motion completed.")
            self.currentState = Task1FSMState.RESUME_EXPLORE_LITE
    
    def handle_resumeExploreLite(self):
        msg = Bool()
        msg.data = True
        self.exploreLiteExploreResumeTopicPublisher.publish(msg)
        self.get_logger().info("[RESUME_EXPLORE_LITE] Published True on 'explore/resume'. ExploreLite should resume (note that ExploreLite may be very slow in resuming due to how it's internally implemented).")
        self.currentState = Task1FSMState.WAIT_EXPLORATION_COMPLETE

    def handle_waitExplorationComplete(self):
        # Temporary stop point until the next FSM state is implemented
        self.fsmTimer.cancel()
        self.get_logger().info(
            "FSM timer cancelled temporarily. Ready to implement next state."
        )

def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = Task1FSMNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None: node.get_logger().info("KeyboardInterrupt received, shutting down task1_fsm_node...")
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__": main()
