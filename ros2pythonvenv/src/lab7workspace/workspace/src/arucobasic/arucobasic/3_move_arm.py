
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
import time

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

        self.get_logger().info("Arm successfully moved to ArUco approach pose!")
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
