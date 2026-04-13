import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from enum import Enum


class Task1FSMState(Enum):
    WAIT_EXPLORE_LITE = 1
    STOP_EXPLORATION = 2


class Task1FSMNode(Node):

    def __init__(self):
        super().__init__("task1_fsm_node")
        self.get_logger().info("Starting task1_fsm_node initialization...")

        # FSM state
        self.current_state = Task1FSMState.WAIT_EXPLORE_LITE

        # Publisher used to stop/resume explore_lite
        self.explore_resume_publisher = self.create_publisher(
            Bool,
            "explore/resume",
            10
        )

        # Periodic timer driving the FSM
        self.fsm_timer = self.create_timer(
            0.5,   # seconds
            self.state_machine_step
        )

        self.get_logger().info(
            "Task1 FSM node initialized. Initial state: WAIT_EXPLORE_LITE."
        )

    def state_machine_step(self):
        if self.current_state == Task1FSMState.WAIT_EXPLORE_LITE:
            self.handle_wait_explore_lite()

        elif self.current_state == Task1FSMState.STOP_EXPLORATION:
            self.handle_stop_exploration()

        else:
            self.get_logger().error(
                f"Unknown FSM state: {self.current_state}"
            )

    def handle_wait_explore_lite(self):
        subscriber_count = self.count_subscribers("explore/resume")

        self.get_logger().info(
            f"[WAIT_EXPLORE_LITE] Subscribers on 'explore/resume': {subscriber_count}"
        )

        if subscriber_count > 0:
            self.get_logger().info(
                "ExploreLite appears to be ready: subscriber detected on 'explore/resume'."
            )
            self.current_state = Task1FSMState.STOP_EXPLORATION

    def handle_stop_exploration(self):
        msg = Bool()
        msg.data = False

        self.explore_resume_publisher.publish(msg)

        self.get_logger().info(
            "[STOP_EXPLORATION] Published False on 'explore/resume'. ExploreLite should stop."
        )

        # For now, stop here until we add the next state
        self.fsm_timer.cancel()
        self.get_logger().info(
            "FSM timer cancelled temporarily. Ready to implement next state."
        )


def main(args=None):
    rclpy.init(args=args)

    node = Task1FSMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down task1_fsm_node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()