import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import PyKDL as kdl

# A very simple example of a TF broadcaster node (that publishes a single static frame)
class FramePublisherNode(Node):

    def __init__(self):

        super().__init__("frame_publisher_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publishFrame()

    def publishFrame(self):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # First of all, we define the parent frame (it must already exist in the TF tree)
        t.header.frame_id = "base_footprint"
        # Secondly, we define the name of the new frame we are customly creating
        t.child_frame_id = "my_new_frame"

        # Define the translation (1m forward in X, 0m in Y, 1m in Z),
        # AKA relative position of the new frame with respect to the parent frame
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0

        # Same for the orientation (in that example, a rotation of π/4 around X)
        rot = kdl.Rotation()
        rot.DoRotX(math.pi / 4)
        q = rot.GetQuaternion()

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Published TF frame '{t.child_frame_id}' relative to '{t.header.frame_id}'!"
        )


def main(args=None):

    rclpy.init(args=args)
    node = FramePublisherNode()
    rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()

if __name__ == "__main__": main()
