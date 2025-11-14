
import rclpy
from rclpy.node import Node
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation
import numpy as np
from sensor_msgs.msg import CameraInfo
from rclpy.wait_for_message import wait_for_message
from geometry_msgs.msg import Point

from . import miscellaneous
class Simple2DPointsSubscriberNode(Node):

    def __init__(self):

        super().__init__('simple_2Dpoints_subscriber_node')
        self.bridge = CvBridge()

        # Subscribing to 2D points topic
        self.tiagoImageSubscription = self.create_subscription(
            Point,
            miscellaneous.customTargetPointTopic,
            self.callback_2Dpoint,
            10
        )

        # Subscribing to Tiago depth topic
        self.tiagoDepthSubscription = self.create_subscription(
            Image,
            miscellaneous.tiagoFrontCameraDepthTopic,
            self.callback_camera_depth,
            10
        )
        self.depth_img = None


        # Getting Tiago camera intrinsic parameters
        self.camera_info_topic = miscellaneous.tiagoFrontCameraIntrinsicParametersTopic
        _, msg = wait_for_message(CameraInfo, self, self.camera_info_topic) # A "simplified subscriber" that asks and waits for only one spin! 
        self.cam_K = np.array(msg.k).reshape(3,3)
        self.cam_D = np.array(list(msg.d))

        self.get_logger().info('2D points subscriber node has been initialized!')

    def callback_camera_depth(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = msg.encoding)

    def callback_2Dpoint(self, msg):
        if self.depth_img is not None:
            point = (msg.x, msg.y)
            print(f'Received 2D point: {point}')
            depth_value = self.depth_img[int(point[1]), int(point[0])]
            Z = depth_value
            X = Z * (point[0] - self.cam_K[0,2]) / self.cam_K[0,0]
            Y = Z * (point[1] - self.cam_K[1,2]) / self.cam_K[1,1]
            self.get_logger().info(f'Detected a 2D point {point} that corresponds to 3D point ({X}, {Y}, {Z}) in camera frame!')

def main():

    rclpy.init() # Initialize the ROS2 Python client library!
    simple_2Dpoints_subscriber_node = Simple2DPointsSubscriberNode() # Create an instance of the custom node defined above

    simple_2Dpoints_subscriber_node.get_logger().info('Waiting for a few seconds before starting the node...')
    time.sleep(3) # Wait for a few seconds to ensure everything is properly initialized (strongly suggested in ROS2...)
    simple_2Dpoints_subscriber_node.get_logger().info('Starting the node now!')
    
    try:
        # Keep processing callbacks (and conseguenting OpenCV GUI) until a Ctrl+C
        rclpy.spin(simple_2Dpoints_subscriber_node)
    except KeyboardInterrupt:
        # User requested shutdown!
        simple_2Dpoints_subscriber_node.get_logger().info('Detected Ctrl+C! Shutting down...')
        pass
    finally:
        try:
            simple_2Dpoints_subscriber_node.get_logger().info('Shutting down 2D points subscriber node...')
            simple_2Dpoints_subscriber_node.destroy_node()
        except Exception:
            pass
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
    

if __name__ == '__main__': main()