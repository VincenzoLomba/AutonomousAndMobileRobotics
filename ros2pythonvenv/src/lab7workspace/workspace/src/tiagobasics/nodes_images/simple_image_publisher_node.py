
import rclpy
from rclpy.node import Node
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from . import miscellaneous
class SimpleImagesPublisherNode(Node):

    def __init__(self):

        super().__init__('simple_images_publisher_node')
        self.bridge = CvBridge()
        self.tiagoImageSubscription = self.create_subscription(
            Image,
            miscellaneous.tiagoCameraTopicName,
            self.callback_camera_image,
            10
        )
        self.image_pub = self.create_publisher(
            Image,
            miscellaneous.flippedCameraTopicName,
            1
        )
        self.get_logger().info('Image publisher node has been initialized!')
    
    def callback_camera_image(self, msg):
        self.get_logger().info(f'New image received from Tiago topic "{miscellaneous.tiagoCameraTopicName}"! Now parsing it...')
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.img_flipped = cv2.flip(img, 0)
        img_msg = self.bridge.cv2_to_imgmsg(self.img_flipped, encoding='bgr8')
        self.image_pub.publish(img_msg)
        self.get_logger().info(f'Flipped image published on topic "{miscellaneous.flippedCameraTopicName}"!')

def main():

    rclpy.init() # Initialize the ROS2 Python client library!
    simple_images_publisher_node = SimpleImagesPublisherNode() # Create an instance of the custom node defined above

    simple_images_publisher_node.get_logger().info('Waiting for a few seconds before starting the node...')
    time.sleep(3) # Wait for a few seconds to ensure everything is properly initialized (strongly suggested in ROS2...)
    simple_images_publisher_node.get_logger().info('Starting the node now!')
    
    try:
        # Keep processing callbacks (and conseguenting OpenCV GUI) until a Ctrl+C
        rclpy.spin(simple_images_publisher_node)
    except KeyboardInterrupt:
        # User requested shutdown!
        simple_images_publisher_node.get_logger().info('Detected Ctrl+C! Shutting down...')
        pass
    finally:
        try:
            simple_images_publisher_node.get_logger().info('Shutting down image publisher node...')
            simple_images_publisher_node.destroy_node()
        except Exception:
            pass
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
    

if __name__ == '__main__': main()