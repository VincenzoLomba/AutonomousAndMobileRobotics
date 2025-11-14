
import rclpy
from rclpy.node import Node
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from . import miscellaneous
class SimpleImagesSubscriberNode(Node):

    def __init__(self):

        super().__init__('simple_images_subscriber_node')
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(
            Image,
            miscellaneous.flippedCameraTopicName,
            self.callback_camera_image, 1
        )
        self.get_logger().info('Image subscriber node has been initialized!')
    
    def callback_camera_image(self, msg):
        self.get_logger().info(f'Image received from topic "{miscellaneous.flippedCameraTopicName}"! Now parsing it...')
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("SimpleSubscriberImage", self.img) # Show the image using OpenCV in a new window called "TiagoCameraImage"
        cv2.waitKey(1) # Needed to refresh the OpenCV window and process GUI events (1 ms delay)

def main():

    rclpy.init() # Initialize the ROS2 Python client library!
    simple_images_subscriber_node = SimpleImagesSubscriberNode() # Create an instance of the custom node defined above

    simple_images_subscriber_node.get_logger().info('Waiting for a few seconds before starting the node...')
    time.sleep(3) # Wait for a few seconds to ensure everything is properly initialized (strongly suggested in ROS2...)
    simple_images_subscriber_node.get_logger().info('Starting the node now!')

    try:
        # Keep processing callbacks (and conseguenting OpenCV GUI) until a Ctrl+C
        rclpy.spin(simple_images_subscriber_node)
    except KeyboardInterrupt:
        # User requested shutdown!
        simple_images_subscriber_node.get_logger().info('Detected Ctrl+C! Shutting down...')
        pass
    finally:
        # Clean up ROS and OpenCV
        try:
            simple_images_subscriber_node.get_logger().info('Shutting down image subscriber node...')
            simple_images_subscriber_node.destroy_node()
        except Exception:
            pass
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
    

if __name__ == '__main__': main()