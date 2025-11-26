from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import GripperInterface

# Gripper parameters
GRIPPER_JOINT_NAMES = [
    "gripper_left_finger_joint",
    "gripper_right_finger_joint",
]

OPEN_GRIPPER_POSITIONS = [0.04, 0.04] # One value for each Tiago gripper joint, 4cm each!
                                      # This is the couple of values that are gonna be used when the "open" method is called!
CLOSED_GRIPPER_POSITIONS = [0.0, 0.0] # Same as before, but for the "close" method (0cm each)!

# Name of the MoveIt planning group that contains the gripper joints.
# This group is defined in the robot's SRDF and is used by MoveIt to know
# which joints belong to the gripper subsystem.
GRIPPER_GROUP_NAME = "gripper"

# ROS2 action used to command the gripper fingers. This is the action
# provided by the gripper's joint trajectory controller, which executes
# the open/close movements by receiving joint trajectories.
# In a nutshell: a "control_msgs/action/FollowJointTrajectory" type ROS2 action supposed to be present and used by the gripper controller node!
GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"


class SimpleMoveITPlannerGripperGoalNode(Node):

    def __init__(self):
        super().__init__("simple_moveitplanner_grippergoal_node")
        self.get_logger().info("Initializing gripper node...")

        self.callback_group = ReentrantCallbackGroup()

        # Create Gripper interface (no MoveIt2 needed!)
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=GRIPPER_JOINT_NAMES,
            open_gripper_joint_positions=OPEN_GRIPPER_POSITIONS,
            closed_gripper_joint_positions=CLOSED_GRIPPER_POSITIONS,
            gripper_group_name=GRIPPER_GROUP_NAME,
            callback_group=self.callback_group,
            gripper_command_action_name=GRIPPER_COMMAND_ACTION_NAME,
        )

        self.get_logger().info("Gripper interface created!")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveITPlannerGripperGoalNode()

    # executor in thread (your style)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.create_rate(1.0).sleep()  # Wait a bit (1 sec)...

    # OPEN
    node.get_logger().info("Opening gripper...")
    node.gripper.open()
    node.gripper.wait_until_executed()

    # CLOSE
    node.get_logger().info("Closing gripper...")
    node.gripper.close()
    node.gripper.wait_until_executed()

    # CUSTOM POSITION
    node.get_logger().info("Moving gripper to 0.02 (2cm)...")
    node.gripper.move_to_position(0.02)
    node.gripper.wait_until_executed()

    node.get_logger().info("Gripper sequence done!")
    rclpy.shutdown()


if __name__ == "__main__": main()
