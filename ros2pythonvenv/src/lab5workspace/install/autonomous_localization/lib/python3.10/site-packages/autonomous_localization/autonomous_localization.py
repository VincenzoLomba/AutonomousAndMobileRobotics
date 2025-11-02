
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import time
import numpy as np
from nav_msgs.msg import Odometry

TWO_PI = 6.28
ROTATION_VELOCITY = -0.6 # Negative value for clockwise rotation (ROS2 convention)

# Define a new custom node that performs self-localization by checking the estimated pose covariance.
# Important: notice that we're gonna use some topics for which we suppose existing some OTHER nodes actually using them.
# In particular, nodes related to TurtleBot3 navigation stack are supposed to be running in parallel to this one.
# Indeed, we suppose that nodes names are the same AND also that the type od shared data per node is also the same.
class InitialPositionNode(Node):
  
    def __init__(self):
        super().__init__('initial_position_node')

        # Subscribe to the amcl_pose topic to get the robot's estimated position (AMCL = Adaptive Monte Carlo Localization)
        self.odom_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self.amcl_callback, 10
        )

        # Create the publisher for the initialpose topic to set the robot's initial position
        self.amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )

        # Subscribe to the odom topic to get the robot's odometry data AKA the robot current position (and orientation) estimate
        self.odometry_subscription = self.create_subscription(
            Odometry, 'odom',
            self.odom_callback, 10
        )

        # Create the publisher for the cmd_vel topic to send velocity commands to the robot
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Parameters initialization
        self.covariance_treshold = 0.07
        self.covariance_msg = PoseWithCovarianceStamped()
        self.covariance_values = PoseWithCovarianceStamped()
        self.tb3_pose = [0, 0, 0]
        self.tb3_orientation = [0, 0, 0, 0]

        self.get_logger().info('Waiting for the initial position...')
        # Check for ACTUALLY present events (messages, timers, services, ...) in the node.
        # If present, process them until there is no more of them AND THEN immediately return.
        # If none of them are present, wait for up to timeout_sec seconds.
        rclpy.spin_once(self, timeout_sec = 1)

    # The callback function for the amcl_pose topic subscription will update the related local variables.
    def amcl_callback(self, msg):
        self.get_logger().warn("AMCL callback in execution...")
        self.amcl_position = msg.pose.pose.position
        self.amcl_orientation = msg.pose.pose.orientation
        self.covariance_msg.pose.covariance = msg.pose.covariance
        self.get_logger().info(f"Actual covariance (x, y, yaw): {self.covariance_msg.pose.covariance[0]}, {self.covariance_msg.pose.covariance[7]}, {self.covariance_msg.pose.covariance[35]}")

    # The callback function for the odom topic subscription will update the related local variables.
    def odom_callback(self, odom_msg):
        x_o = odom_msg.pose.pose.orientation.x
        y_o = odom_msg.pose.pose.orientation.y
        z_o = odom_msg.pose.pose.orientation.z
        w_o = odom_msg.pose.pose.orientation.w
        # With the following five lines we are initializing the covariance values and, indeed, setting uncertainty.
        # This is indeed interesting because AMCL will use these values as INITIAL uncertainty for the localization process.
        # Indeed, in here we're talking about data that will be used in the initialization part.
        # The covariance matrix is 6x6, but represented as a flat array of 36 elements with order [x, y, z, roll, pitch, yaw].
        # Relevant indexes are: 0 (x position), 7 (y position), 35 (z orientation)
        # In general:
        # cov[0] = var(x)
        # cov[7] = var(y)
        # cov[14] = var(z)
        # cov[21] = var(roll)
        # cov[28] = var(pitch)
        # cov[35] = var(yaw)
        self.get_logger().info("Odom callback in execution...")
        cov = np.zeros(36, dtype=np.float64)
        cov[0] = 200
        cov[7] = 200
        cov[35] = 1
        self.covariance_values.pose.covariance = cov
        self.tb3_orientation = [x_o, y_o, z_o, w_o]

    # Publish the initial guess position on the topic initialpose
    def publish_initial_pose(self):
        rclpy.spin_once(self, timeout_sec = 1)
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.pose.pose.position.x = float(self.tb3_pose[0])
        initial_pose_msg.pose.pose.position.y = float(self.tb3_pose[1])
        initial_pose_msg.pose.pose.position.z = float(self.tb3_pose[2])
        initial_pose_msg.pose.pose.orientation.x = float(self.tb3_orientation[0])
        initial_pose_msg.pose.pose.orientation.y = float(self.tb3_orientation[1])
        initial_pose_msg.pose.pose.orientation.z = float(self.tb3_orientation[2])
        initial_pose_msg.pose.pose.orientation.w = float(self.tb3_orientation[3])
        initial_pose_msg.pose.covariance = self.covariance_values.pose.covariance
        self.get_logger().info(f"Initial covariance (x, y, yaw): {initial_pose_msg.pose.covariance[0]}, {initial_pose_msg.pose.covariance[7]}, {initial_pose_msg.pose.covariance[35]}")
        initial_pose_msg.header.frame_id = 'map' # Indicate the reference frame for the pose (map is for the global reference frame)
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.amcl_pose_publisher.publish(initial_pose_msg)

    def localization(self):
        while True:
            self.get_logger().info('Localization in progress...')
            self.rotate()
            rclpy.spin_once(self, timeout_sec=1)
            if self.check_covariance(): break

    def check_covariance(self):
        covariance_values = self.covariance_msg.pose.covariance
        if np.max(covariance_values) < self.covariance_treshold:
            self.get_logger().info("Covariance below the threshold.")
            self.get_logger().info("Robot is localized!")
            return True
        else:
            self.get_logger().warn("Covariance above the threshold.")
            return False
        
    def rotate(self):
        vel_msg = Twist()
        vel_msg.angular.z = ROTATION_VELOCITY
        self.publisher.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)
        time.sleep(abs(TWO_PI / vel_msg.angular.z)) # Rotate for a full circle!
        self.stop()

    def stop(self):
        vel_msg = Twist()
        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0.0
        self.publisher.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)

def main():
    rclpy.init() # Initialize the ROS2 Python client library!
    initial_position_node = InitialPositionNode() # Create an instance of the custom node defined above
    time.sleep(5) # Wait for a few seconds to ensure everything is properly initialized (strongly suggested in ROS2...)
    initial_position_node.publish_initial_pose()
    initial_position_node.localization()
    initial_position_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()