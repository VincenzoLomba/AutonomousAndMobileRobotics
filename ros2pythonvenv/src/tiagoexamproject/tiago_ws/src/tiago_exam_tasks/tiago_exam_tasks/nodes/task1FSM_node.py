
# This Python files defines a ROS2 Humble Node which implements the FSM that implements the logic that executes the Task1

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from enum import Enum

class Task1FSMState(Enum):
    WAIT_FOR_EXPLORE_LITE = 1
    PAUSE_EXPLORE_LITE = 2


class Task1FSMNode(Node):

    def __init__(self):

        super().__init__("task1_fsm_node")
        self.get_logger().info("Starting task1_fsm_node initialization...")
        self.exploreLiteExploreResumeTopicLabel = "explore/resume"
        self.exploreLiteNodeEndpointname = "explore_node"

        self.exploreLiteExploreResumeTopicPublisher = self.create_publisher(Bool, self.exploreLiteExploreResumeTopicLabel, 10) # remark: 10 is the length of this publisher queue, i.e. the maximum number of messages that can be buffered before being sent out (if the subscriber is not receiving & processing them fast enough)

        self.currentState = Task1FSMState.WAIT_FOR_EXPLORE_LITE
        self.fsmTimer = self.create_timer(0.5, self.stepUpFSM) # A timer created like that (alias with self.create_timer) is gonna be spun as soon as this node is spun via rclpy.spin()
        self.get_logger().info(f"Task1 FSM node initialized. Initial state: {self.currentState.name}.")

    def stepUpFSM(self):
        if self.currentState == Task1FSMState.WAIT_FOR_EXPLORE_LITE: self.handle_waitForExploreLite()
        elif self.currentState == Task1FSMState.PAUSE_EXPLORE_LITE: self.handle_pauseExploreLite()
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