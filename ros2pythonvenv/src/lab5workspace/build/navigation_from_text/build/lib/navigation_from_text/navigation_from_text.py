
import rclpy
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle
import time
from rclpy.node import Node
import os
from geometry_msgs.msg import PoseStamped
from tf2_ros import Duration
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

# Defining a new custom node that reads a list of navigation goals from a text file and sends them to the Nav2 stack one by one.
# The basic idea is actually to move to all goals, one at a time.
# In order to to that we use the navigate_to_pose action provided by the Nav2 stack!
class NavigatorFromTextfile(Node):

    def __init__(self):

        super().__init__('action_server_control')
        print('Initializing node...')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_to_pose_client.wait_for_server()
        print('Init completed!')

    def goToPose(self, pose: PoseStamped):

        self.debug("Waiting for ’NavigateToPose’ action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("’NavigateToPose’ server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback) # Calling the Action and also setting a proper feedback callback function for it
        rclpy.spin_until_future_complete(self, send_goal_future) # Execute work until the future send_goal_future is complete.
                                                                 # Callbacks and other work will be executed by the provided executor until
                                                                 # future.done() returns True or the context associated with the executor
                                                                 # is shutdown.
                                                                 # In other words: we spin until the action server accept or reject our goal.
        self.goal_handle: ClientGoalHandle = send_goal_future.result() # Get the result of the future (which is the goal handle)
        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + ' was rejected!')
            return False
        self.result_future: Future = self.goal_handle.get_result_async()
        return True
    
    def isNavComplete(self):
        if not self.result_future: # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else: # Timed out, still processing, not complete yet
            return False
        return True

    def getFeedback(self): return self.feedback
    def getResult(self): return self.status

    def waitUntilNav2Active(self):
        self.info('Wait for Nav2 to be ready...')
        self._waitForNodeToActivate('amcl')
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return
    
    def _waitForNodeToActivate(self, node_name):
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')
        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return
    
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
    def info(self, msg):
        self.get_logger().info(msg)
        return
    def warn(self, msg):
        self.get_logger().warn(msg)
        return
    def error(self, msg):
        self.get_logger().error(msg)
        return
    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def navigation(self):
        i = 0
        while not self.isNavComplete():
            i = i + 1
            feedback = self.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0): self.cancelNav() # Cancel navigation if it takes more than 10 minutes
        result = self.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            self.info('Goal was canceled!')
        elif result == GoalStatus.STATUS_ABORTED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

def main(args=None):
    rclpy.init()
    navigator = NavigatorFromTextfile()
    navigator.waitUntilNav2Active()
    path_to_textfile = "/home/vincenzo/Documenti/AMR/ros2pythonvenv/src/lab5workspace/data/goals.txt"
    if os.path.exists(path_to_textfile):
        with open(path_to_textfile, "r") as file:
            lines = file.readlines()
            for line in lines:
                goal = line.split()
                goal_x = float(goal[0])
                goal_y = float(goal[1])
                goal_w = float(goal[2])
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg() # Get the rclpy clock associated to the node and ask for actual time, then convert it to ROS2 msg
                                                                              # It is good practice to always fill the header stamp with the current time when publishing a PoseStamped message
                                                                              # Using the node clock ensures that the time is coherent with the rest of the system (especially when using simulation)
                                                                              # Also, the node is expected to use /use_sim_time = True parameter when working with simulation
                goal_pose.pose.position.x = float(goal_x)
                goal_pose.pose.position.y = float(goal_y)
                goal_pose.pose.orientation.w = float(goal_w)
                navigator.goToPose(goal_pose)
                navigator.navigation()
                time.sleep(2) # wait a little before sending the next goal
            navigator.info("All the " + str(len(lines)) + " goals processed")
    else:
        navigator.info("The specified path doesn’t exist")

if __name__ == '__main__': main()