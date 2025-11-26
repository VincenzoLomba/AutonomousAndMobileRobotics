
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import PyKDL as kdl
import time

class ArucoTFSubscriber(Node):

    def __init__(self):

        super().__init__('aruco_tf_subscriber')

        # Create a subscription to the TransformStamped topic of Aruco marker
        self.subscription = self.create_subscription(
            TransformStamped,
            '/aruco_single/transform',
            self.listener_callback,
            1
        )

        self.get_logger().info("Aruco TF subscriber started and listening...")

    def listener_callback(self, msg: TransformStamped):
        
        # Extract translation
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        z = msg.transform.translation.z

        # Extract quaternion
        qx = msg.transform.rotation.x
        qy = msg.transform.rotation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w

        # Construct the PyKDL frame
        position = kdl.Vector(x, y, z)
        rotation = kdl.Rotation.Quaternion(qx, qy, qz, qw)
        frame = kdl.Frame(rotation, position)

        # Print the result
        self.get_logger().info(f"KDL Frame:\n{frame}")


def main():
    
    rclpy.init() # Initialize the ROS2 Python client library!
    node = ArucoTFSubscriber()

    node.get_logger().info('Waiting for a few seconds before starting the node...')
    time.sleep(3) # Wait for a few seconds to ensure everything is properly initialized (strongly suggested in ROS2...)
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


if __name__ == '__main__': main()