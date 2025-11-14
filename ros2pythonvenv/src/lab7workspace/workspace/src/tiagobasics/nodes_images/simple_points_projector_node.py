
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
class SimplePointsProjectorNode(Node):

    def __init__(self):

        super().__init__('simple_points_projector_node')
        self.bridge = CvBridge()

        # Subscribing to Tiago camera topic
        self.tiagoImageSubscription = self.create_subscription(
            Image,
            miscellaneous.tiagoCameraTopicName,
            self.callback_camera_image,
            10
        )
        
        # Preparing publisher for elaborated (with projected points) images
        self.image_pub = self.create_publisher(
            Image,
            miscellaneous.flippedCameraTopicName,
            1
        )

        # Preparing publisher for generated 2D points (AKA for projected 3D points)
        self.points_pub = self.create_publisher(
            Point,
            miscellaneous.customTargetPointTopic,
            1
        )

        # Getting Tiago camera intrinsic parameters
        self.camera_info_topic = miscellaneous.tiagoFrontCameraIntrinsicParametersTopic
        _, msg = wait_for_message(CameraInfo, self, self.camera_info_topic) # A "simplified subscriber" that asks and waits for only one spin! 
        self.cam_K = np.array(msg.k).reshape(3,3)
        self.cam_D = np.array(list(msg.d))

        # Setting up the TF2 listener (to get the transformations between the gripper frame and the camera frame)
        self.tf_buffer = Buffer() # Create the TF2 buffer (this is a tf2_ros.buffer object that has the purpose to be used to work with transformations)
        self.tf_listener = TransformListener(self.tf_buffer, self) # Create the TF2 listener (this is a tf2_ros.transform_listener object that will fill the buffer with the desired transformations)
        self.timer = self.create_timer(0.1, self.timerCallback)

        self.get_logger().info('Points projector node has been initialized!')

    def callback_camera_image(self, msg):
        self.get_logger().info(f'New image received from Tiago topic "{miscellaneous.tiagoCameraTopicName}"! Now parsing it (adding gripper origin)...')
        if not hasattr(self, 't'):
            self.get_logger().warn('Transform not available yet! No parse of the image can be done!')
        else:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            point2d = self.projectAPoint(np.array([[0, 0, 0]]), self.homogeneous_matrix_from_transform(self.t), self.cam_K, self.cam_D)
            self.points_pub.publish(Point(x=float(point2d[0]), y=float(point2d[1]), z=0.0))
            self.get_logger().info(f'World point (0,0,0) projected at pixel coordinates {point2d} (2D point publiched on topic "{miscellaneous.customTargetPointTopic}")!')
            cv2.circle(img, (int(point2d[0]), int(point2d[1])), 5, (0, 255, 0), -1)
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.image_pub.publish(img_msg)
            self.get_logger().info(f'With-a-point image published on topic "{miscellaneous.flippedCameraTopicName}"!')

    def timerCallback(self):
        try:
            self.t = self.tf_buffer.lookup_transform(    # This method will lookup the transformation between two frames at a given time
                miscellaneous.tiagoFrontCameraFrameName, # Target frame
                miscellaneous.gripperFrameName,          # Source frame
                rclpy.time.Time()                        # Time at which the transform is desired (0 means the latest available)
            )
        except TransformException as ex:
            # Log the exception message for easier debugging
            self.get_logger().info('Could not transform: %s' % str(ex))
        return
    
    def homogeneous_matrix_from_transform(self, t):
        position = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])
        quat = [
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ]
        Tmatrix = np.eye(4)
        Tmatrix[:3, 3] = position
        Tmatrix[:3, :3] = Rotation.from_quat(quat).as_matrix()
        return Tmatrix
    
    def projectAPoint(self, points3d, camera_pose, cam_K, cam_D):
        # Requested arguments:
        # - points3d: one or more points in the 3D world expressed w.r.t. the Tiago Gripper frame
        # - camera_pose: the homogeneous transformation FROM the frame in which points3d are expressed TO the Tiago front camera reference frame
        # - cam_K: the intrinsic of the camera
        # - cam_D: distorsion parameters of the camera
        R, _ = cv2.Rodrigues(camera_pose[:3, :3])
        t = camera_pose[:3, 3]
        points3d = points3d.reshape(-1, 3).astype(np.float32)
        points2d, _ = cv2.projectPoints(points3d, R, t, cam_K, cam_D)
        return points2d.squeeze()

def main():

    rclpy.init() # Initialize the ROS2 Python client library!
    simple_points_projector_node = SimplePointsProjectorNode() # Create an instance of the custom node defined above

    simple_points_projector_node.get_logger().info('Waiting for a few seconds before starting the node...')
    time.sleep(3) # Wait for a few seconds to ensure everything is properly initialized (strongly suggested in ROS2...)
    simple_points_projector_node.get_logger().info('Starting the node now!')
    
    try:
        # Keep processing callbacks (and conseguenting OpenCV GUI) until a Ctrl+C
        rclpy.spin(simple_points_projector_node)
    except KeyboardInterrupt:
        # User requested shutdown!
        simple_points_projector_node.get_logger().info('Detected Ctrl+C! Shutting down...')
        pass
    finally:
        try:
            simple_points_projector_node.get_logger().info('Shutting down image publisher node...')
            simple_points_projector_node.destroy_node()
        except Exception:
            pass
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
    

if __name__ == '__main__': main()