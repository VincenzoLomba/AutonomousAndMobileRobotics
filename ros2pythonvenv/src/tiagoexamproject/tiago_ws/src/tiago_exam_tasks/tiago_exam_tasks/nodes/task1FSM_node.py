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

        self.current_state = Task1FSMState.WAIT_EXPLORE_LITE

        self.explore_resume_publisher = self.create_publisher(
            Bool,
            "explore/resume",
            10
        )

        self.fsm_timer = self.create_timer(
            0.5,
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
            self.get_logger().error(f"Unknown FSM state: {self.current_state}")

    def is_explore_lite_ready(self) -> bool:
        subscriptions_info = self.get_subscriptions_info_by_topic("explore/resume")

        if not subscriptions_info:
            return False

        for endpoint in subscriptions_info:
            self.get_logger().info(
                "[WAIT_EXPLORE_LITE] Found subscriber on 'explore/resume' -> "
                f"name: {endpoint.node_name}, "
                f"namespace: {endpoint.node_namespace}, "
                f"type: {endpoint.topic_type}"
            )

            if endpoint.node_name == "explore_node":
                return True

        return False

    def handle_wait_explore_lite(self):
        if self.is_explore_lite_ready():
            self.get_logger().info(
                "ExploreLite is ready: subscriber 'explore_node' detected on 'explore/resume'."
            )
            self.current_state = Task1FSMState.STOP_EXPLORATION
        else:
            self.get_logger().info(
                "[WAIT_EXPLORE_LITE] ExploreLite subscriber not detected yet."
            )

    def handle_stop_exploration(self):
        msg = Bool()
        msg.data = False

        self.explore_resume_publisher.publish(msg)

        self.get_logger().info(
            "[STOP_EXPLORATION] Published False on 'explore/resume'. ExploreLite should stop."
        )

        # Temporary stop point until the next FSM state is implemented
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