
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import PyKDL as kdl
import time

# A simple Node thhat listens to the ArUco marker TF and broadcasts two new TF related to it!
class ArucoTFBroadcaster(Node):

    def __init__(self):

        super().__init__('aruco_tf_broadcaster')
        self.get_logger().info("Initializing Aruco TF Broadcaster Node...")

        # Subscription to ArUco marker TF
        self.subscription = self.create_subscription(
            TransformStamped,
            '/aruco_single/transform',
            self.marker_callback,
            1
        )

        # Defining the TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Defining the TF2 listener (to get the transformation between base_link and camera frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Aruco TF Broadcaster started!")

    def marker_callback(self, msg):

        # Getting the TF from camera to marker (given by Aruco Node)
        camToMarker = self.conversionTFtoKDL(msg)

        try:
            baseToCameraMSG = self.tf_buffer.lookup_transform(
                'base_link',
                'head_front_camera_rgb_optical_frame',
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        baseToCamera = self.conversionTFtoKDL(baseToCameraMSG)
        baseToTarget = baseToCamera * camToMarker

        offset = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.5))
        baseToApproach = baseToTarget * offset

        # Let's publish the two new TFs!
        self.publishKdlTF(baseToTarget, "base_link", "aruco_marker_target_frame")
        self.publishKdlTF(baseToApproach, "base_link", "aruco_marker_approach_frame")

    def conversionTFtoKDL(self, msg):

        t = msg.transform.translation
        r = msg.transform.rotation
        pos = kdl.Vector(t.x, t.y, t.z)
        rot = kdl.Rotation.Quaternion(r.x, r.y, r.z, r.w)
        return kdl.Frame(rot, pos)


    def publishKdlTF(self, frame, parent, child):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child

        pos = frame.p
        msg.transform.translation.x = pos[0]
        msg.transform.translation.y = pos[1]
        msg.transform.translation.z = pos[2]

        rot = frame.M.GetQuaternion()
        msg.transform.rotation.x = rot[0]
        msg.transform.rotation.y = rot[1]
        msg.transform.rotation.z = rot[2]
        msg.transform.rotation.w = rot[3]

        self.tf_broadcaster.sendTransform(msg)


def main(args=None):

    rclpy.init(args=args)
    node = ArucoTFBroadcaster()

    node.get_logger().info('Waiting for a few seconds before starting the node...')
    time.sleep(3)
    node.get_logger().info('Starting the node now!')

    try:
        # Keep processing callbacks (and conseguenting OpenCV GUI) until a Ctrl+C
        rclpy.spin(node)
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
