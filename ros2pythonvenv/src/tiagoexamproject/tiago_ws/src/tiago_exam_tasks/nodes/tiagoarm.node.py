
# This Python files defines a ROS2 Humble Node which exposes an Action Server to be used to control the Tiago's arm

from rclpy.node import Node
from rclpy.action import ActionServer
from tiago_exam_interfaces.action import TiagoArmAction
from . import nodesParameters

class TiagoArmNode(Node):

    def __init__(self):
        
        super().__init__('tiago_arm_node')
        self.get_logger().info("Starting tiago_arm_node initialization...")

        # Initializing the Action Server for controlling the Tiago's arm that this Node is gonna expose
        self._action_server = ActionServer(
            self,
            TiagoArmAction,
            nodesParameters.tiagoArmActionName,
            self.execute_callback
        )

def main(args=None):
    return
    # TODO define a correct main

if __name__ == "__main__": main()