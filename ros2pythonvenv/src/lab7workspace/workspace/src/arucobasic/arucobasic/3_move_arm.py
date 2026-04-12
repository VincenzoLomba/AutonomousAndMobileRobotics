
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import PyKDL as kdl
import math
from pymoveit2 import GripperInterface

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Tiago Parameters
JOINT_NAMES = [
"torso_lift_joint",
"arm_1_joint",
"arm_2_joint",
"arm_3_joint",
"arm_4_joint",
"arm_5_joint",
"arm_6_joint",
"arm_7_joint",
"arm_tool_joint",
]
BASE_LINK_NAME = "base_link"
END_EFFECTOR_NAME = "arm_tool_link"
GROUP_NAME = "arm_torso"

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


# A simple Node that moves the robotic arm to the ArUco marker approach pose
class MoveArmToAruco(Node):

    def __init__(self):

        super().__init__("move_arm_to_aruco")
        self.get_logger().info("Initializing TF listener...")

        # Defining the TF2 listener (to get the transformation between base_link and camera frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Initializing MoveIT2...")

        # Callback group for MoveIt2 (allows parallel callbacks)
        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK_NAME,
            end_effector_name=END_EFFECTOR_NAME,
            group_name=GROUP_NAME,
            callback_group=self.callback_group,
        )

        # Setting up a default planner (this is the Random Rapidly-exploring Tree - bidirectional)
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Setting up a timer to periodically check for the ArUco approach pose TF AND conseguently move the arm to it!
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.executed = False

        # Create Gripper interface (no MoveIt2 needed!)
        # self.callback_group2 = ReentrantCallbackGroup()
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

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Also defined TF broadcaster!")

    def timer_callback(self):

        if self.executed: return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                "base_link",
                "aruco_marker_approach_frame",
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF not available yet: {e}")
            return

        pos = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        position = [pos.x, pos.y, pos.z]
        quat_xyzw = [rot.x, rot.y, rot.z, rot.w]


        self.get_logger().info("Received approach pose, now planning and executing motion with MoveIT2...")

        # Imposta il target e pianifica
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()
        time.sleep(2)


        R = kdl.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w)
        R_extra = kdl.Rotation.RotY(math.pi / 2.0)
        R_new = R * R_extra
        qx, qy, qz, qw = R_new.GetQuaternion()
        quat_xyzw = [qx, qy, qz, qw]
        F_approach = kdl.Frame(
            R_new,
            kdl.Vector(pos.x, pos.y, pos.z)
        )
        F_offset = kdl.Frame(kdl.Vector(0.3, 0.0, 0.0))
        F_object = F_approach * F_offset

        self.get_logger().info("Received approach pose, now planning and executing motion with MoveIT2...")

        # Imposta il target e pianifica
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Arm successfully moved to ArUco approach pose!")

        self.get_logger().info("Now opening the gripper...")
        self.get_logger().info("Opening gripper...")
        self.gripper.open()
        self.gripper.wait_until_executed()

         # Imposta il target e pianifica
        p_obj = F_object.p
        R_obj = F_object.M

        qx, qy, qz, qw = R_obj.GetQuaternion()
        position_obj = [p_obj[0], p_obj[1], p_obj[2]]
        quat_xyzw_obj = [qx, qy, qz, qw]
        self.moveit2.move_to_pose(position=position_obj, quat_xyzw=quat_xyzw_obj)
        self.moveit2.wait_until_executed()
        time.sleep(2)

        self.executed = True

def main():
    rclpy.init()
    node = MoveArmToAruco()

    node.get_logger().info('Waiting for a few seconds before starting the node...')
    time.sleep(3)
    node.get_logger().info('Starting the node now!')

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        # Keep processing callbacks (and conseguenting OpenCV GUI) until a Ctrl+C
        executor.spin()
    except KeyboardInterrupt:
        # User requested shutdown!
        node.get_logger().info('Detected Ctrl+C! Shutting down...')
        pass
    finally:
        try:
            node.get_logger().info('Shutting down image publisher node...')
            node.destroy_node()
        except Exception: pass
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
