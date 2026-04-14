
# This Python files defines a ROS2 Humble Node which exposes an Action Server to be used to control the Tiago's arm

from rclpy.node import Node
from rclpy.action import ActionServer
from tiago_exam_interfaces.action import TiagoArm
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
import rclpy
from threading import Thread
from . import nodesParameters

# TODO: manage the possibility (unwanted, but possible) of receiving a new goal while one is already being executed

# Note: you may issue "ros2 launch tiago_moveit_config moveit_rviz.launch.py" to open the RViz page related to MoveIT to check for these values
JOINT_NAMES = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint"
]
BASE_LINK_NAME = "base_link"
END_EFFECTOR_NAME = "arm_tool_link"
GROUP_NAME = "arm_torso"

class TiagoArmNode(Node):

    def __init__(self):
        
        super().__init__('tiago_arm_node')
        self.get_logger().info("Starting tiago_arm_node initialization...")

        # ReentrantCallbackGroup allows callbacks associated with the same node/group to be executed without strict mutual exclusion, which is useful here because
        # indeed within this note the action server and the internal ROS interfaces used by MoveIt2 may need to make progress concurrently.
        # This choice only becomes truly meaningful when the node is spun by a MultiThreadedExecutor (i.e. with more than one worker thread);
        # indeed, with a single-threaded executor, callbacks are still processed one at a time.
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node = self, # The ROS2 node that MoveIt2 will attach to.
                         # All necessary internal components (action clients, service, clients, TF lookups, publishers/subscribers)
                         # are gonna be created under this node.
            joint_names = JOINT_NAMES, # Ordered list of joint names belonging to the planning group.
                                       # MoveIt2 expects these names to match the robot’s URDF/SRDF configuration.
                                       # For TIAGO, they are as listed above.
            base_link_name = BASE_LINK_NAME, # The root frame of the kinematic chain / manipulator.
                                             # For TIAGO, this is typically "base_link".
                                             # All poses are gonna be interpreted relative to this frame.
            end_effector_name = END_EFFECTOR_NAME, # The frame at the tip of the kinematic chain / manipulator.
                                                   # (e.g., the wrist or tool frame).
                                                   # MoveIt2 will plan motions to move this frame to the target poses you specify.
                                                   # For TIAGO, this is typically "arm_tool_link".
            group_name=GROUP_NAME, # The MoveIt2 “planning group” name.
                                   # Must match one of the groups defined in the SRDF file, such as "arm_torso" for TIAGO.
            callback_group = self.callback_group, # Callback group controlling how ROS callbacks (action feedback, service responses, timers, TF) are gonna be executed.
                                                  # Again, the ReentrantCallbackGroup allows multiple callbacks to run in parallel and is required for MoveIt2 to work correctly.
        )
        # Setting up a default planner to be used within MoveIT (used the Random Rapidly-exploring Tree in Bidirectional Version)
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.get_logger().info("MoveIt2 interface successfully created. Planner set to RRTConnectkConfigDefault.")

        # Initializing the Action Server for controlling the Tiago's arm that this Node is gonna expose
        self.actionServer = ActionServer(
            self,
            TiagoArm,
            nodesParameters.tiagoArmActionName,
            execute_callback = self.actionServerExecuteCallback,
            callback_group = self.callback_group,
        )
        self.get_logger().info(f"TiagoArm Action Server is ready on action name '{nodesParameters.tiagoArmActionName}'.")

    def actionServerExecuteCallback(self, goal_handle):

        self.get_logger().info("Received new TiagoArm goal.")
        feedbackMessage = TiagoArm.Feedback()
        result = TiagoArm.Result()

        try:
            jointPositions = list(goal_handle.request.joint_positions)

            if len(jointPositions) != len(JOINT_NAMES):
                msg = f"Invalid TiagoArm goal: expected {len(JOINT_NAMES)} joint values (in the request), BUT received {len(jointPositions)}."
                self.get_logger().error(msg)
                feedbackMessage.current_state = "invalid_goal"
                goal_handle.publish_feedback(feedbackMessage)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result

            self.get_logger().info(f"Planning & waiting for execution by MoveIt2 of the motion to the specified joint configuration: {jointPositions}")
            feedbackMessage.current_state = "planning"
            goal_handle.publish_feedback(feedbackMessage)
            self.moveit2.move_to_configuration(jointPositions)
            feedbackMessage.current_state = "executing"
            goal_handle.publish_feedback(feedbackMessage)
            execution_result = self.moveit2.wait_until_executed()

            if execution_result is False:
                msg = "Motion execution failed."
                self.get_logger().error(msg)
                feedbackMessage.current_state = "failed"
                goal_handle.publish_feedback(feedbackMessage)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result
            
            feedbackMessage.current_state = "completed"
            goal_handle.publish_feedback(feedbackMessage)
            goal_handle.succeed()
            result.success = True
            result.message = "Motion completed successfully."
            self.get_logger().info(result.message)
            return result

        except Exception as exc:
            msg = f"Exception while executing TiagoArm action: {exc}"
            self.get_logger().error(msg)
            feedbackMessage.current_state = "exception"
            goal_handle.publish_feedback(feedbackMessage)
            goal_handle.abort()
            result.success = False
            result.message = msg
            return result

def main(args=None):

    rclpy.init(args=args)
    node = TiagoArmNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads = 2) # Create a ROS2 executor that can run callbacks in parallel using 2 worker threads.
                                                                      # This is required because MoveIt2 uses multiple callbacks at the same time (action feedback, planning service responses, TF lookups, etc.).
    executor.add_node(node) # Register this very TiagoArmNode with the executor so that all its subscriptions,
                            # timers, services and action clients become active and start receiving events and callbacks.
    spin_thread = Thread(target = executor.spin, daemon = True, args = ()) # Create a new Python thread that will run executor.spin().
                                                                           # Note that executor.spin() is a *blocking* call:
                                                                           # it enters an infinite loop and continuously handles ROS2 callbacks.
                                                                           # Running spin() in a separate thread allows the main thread to continue executing the remaining code.
    spin_thread.start() # Start the executor thread.
                        # From this point, ROS2 is actively processing callbacks in the background.

    node.get_logger().info("TiagoArm node now spinning with MultiThreadedExecutor.")

    try:
        spin_thread.join() # Wait for the executor thread to finish (which is basically waiting indefinitely until the node is shut down)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down TiagoArm node...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__": main()
