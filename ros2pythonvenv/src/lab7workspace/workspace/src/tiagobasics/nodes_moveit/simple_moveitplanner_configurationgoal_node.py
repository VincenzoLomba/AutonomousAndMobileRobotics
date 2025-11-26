
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2

# Tiago Parameters
JOINT_NAMES = [
"torso_lift_joint",
"arm_1_joint",
"arm_2_joint",
"arm_3_joint",
"arm_4_joint",
"arm_5_joint",
"arm_6_joint",
"arm_7_joint",
"arm_tool_joint",
]
BASE_LINK_NAME = "base_link"
END_EFFECTOR_NAME = "arm_tool_link"
GROUP_NAME = "arm_torso"

class SimpleMoveITPlannerConfigurationGoalNode(Node):

    def __init__(self):
        
        super().__init__('simple_moveit_planner_configurationgoal_node')
        self.get_logger().info("Starting node initialization...")

        # Callback group for MoveIt2 (allows parallel callbacks)
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,  # The ROS2 node that MoveIt2 will attach to.
                        # All internal components (action clients, service
                        # clients, TF lookups, publishers/subscribers)
                        # are created under this node.
            joint_names=JOINT_NAMES, # Ordered list of joint names belonging to the
                                     # planning group. MoveIt2 expects these names to
                                    # match the robot’s URDF/SRDF configuration.
            base_link_name=BASE_LINK_NAME, # The root frame of the kinematic chain.
                                           # For TIAGO, this is typically "base_link".
                                           # All poses are interpreted relative to this frame.
            end_effector_name=END_EFFECTOR_NAME, # The frame at the tip of the manipulator
                                                 # (e.g., the wrist or tool frame).
                                                 # MoveIt2 will plan motions to move this frame
                                                 # to the target poses you specify.
            group_name=GROUP_NAME, # The MoveIt2 “planning group” name.
                                   # Must match one of the groups defined in the SRDF,
                                   # such as "arm_torso" for TIAGO.
            callback_group=self.callback_group, # Callback group controlling how ROS callbacks
                                                # (action feedback, service responses, timers, TF)
                                                # are executed. ReentrantCallbackGroup allows
                                                # multiple callbacks to run in parallel and is
                                                # required for MoveIt2 to work correctly.
        )

        # Setting up a default planner (this is the Random Rapidly-exploring Tree - bidirectional)
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        self.get_logger().info("MoveIt2 interface created!")

def main(args=None):
    
    rclpy.init(args=args)
    node = SimpleMoveITPlannerConfigurationGoalNode()
    
    node.get_logger().info('Starting main!')

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2) # Create a ROS2 executor that can run callbacks
                                                                    # in parallel using 2 worker threads.
                                                                    # This is required because MoveIt2 uses multiple
                                                                    # callbacks at the same time (action feedback,
                                                                    # planning service responses, TF lookups, etc.).
    executor.add_node(node) # Register this node with the executor so that
                            # all its subscriptions, timers, services,
                            # and action clients become active and start
                            # receiving events and callbacks.
    spin_thread = Thread(target=executor.spin, daemon=True, args=()) # Create a new Python thread that will run
                                                                     # executor.spin().
                                                                     #
                                                                     # executor.spin() is a *blocking* call:
                                                                     # it enters an infinite loop and continuously
                                                                     # handles ROS2 callbacks.
                                                                     #
                                                                     # Running spin() in a separate thread allows
                                                                     # the main thread to continue executing user's
                                                                     # code (e.g., sending MoveIt2 commands).
    spin_thread.start() # Start the executor thread.
                        # From this point, ROS2 is actively processing
                        # callbacks in the background while the main
                        # node logic continues.

    node.create_rate(1.0).sleep() # Small delay to ensure the executor has started before sending the first MoveIt2 commands.
                                  # Indeed, sleeping for 1 second.
                                  # This is NOT a timer: it simply pauses the current thread to give the executor time to start.
                                  # Sort of a time.sleep(1) but using ROS2's rate mechanism!

    node.get_logger().info('Threaded executor started! Now sending MoveIt2 configuration goal command...')

    # Actually moving to a desired configuration goal thanks to MoveIt2:
    # joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_positions = [0.0] * len(JOINT_NAMES)
    node.moveit2.move_to_configuration(joint_positions) # Send the command to MoveIt2 to move the end-effector
                                                                      # (starting the movement asynchronously in the background, non blocking)
    node.moveit2.wait_until_executed()                                # Blocking wait until the motion is fully executed

    node.get_logger().info('Simple MoveIT planner node has been initialized and moved to the desired pose!')

    rclpy.shutdown()

if __name__ == "__main__": main()