
# This Python files defines a ROS2 Humble Node which exposes an Action Server to be used to control the Tiago's arm

from rclpy.node import Node
from rclpy.action import ActionServer
from tiago_exam_interfaces.action import TiagoArm
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
import rclpy
import math
from threading import Thread
from . import tiagoExamParameters as params

# TODO: manage the possibility (unwanted, but possible) of receiving a new goal while one is already being executed

# Note: you may issue "ros2 launch tiago_moveit_config moveit_rviz.launch.py" to open the RViz page related to MoveIT to check for these values
# In case of a goal in terms of a configuration, that goal will also include the Tiago torso. In that case, the JOINTS names will be:
JOINT_NAMES_WITH_TORSO = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint"
]
# In case of a goal in terms of a pose, the goal will NOT include the Tiago torso. In that case, the JOINTS names will be:
ARM_ONLY_JOINT_NAMES = [
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
# In case of a goal in terms of a configuration, that goal will also include the Tiago torso. In that case, the GROUP name will be:
GROUP_NAME_WITH_TORSO = "arm_torso"
# In case of a goal in terms of a pose, the goal will NOT include the Tiago torso. In that case, the GROUP name will be:
ARM_ONLY_GROUP_NAME = "arm"

class TiagoArmNode(Node):

    def __init__(self):
        
        super().__init__('tiago_arm_node')
        self.get_logger().info("Starting tiago_arm_node initialization...")

        # ReentrantCallbackGroup allows callbacks associated with the same node/group to be executed without strict mutual exclusion,
        # which is useful here because indeed within this node the Action Server and the internal ROS interfaces used by MoveIt2 may need to make progress concurrently.
        # This choice only becomes truly meaningful when the node is spun by a MultiThreadedExecutor (i.e. with more than one worker thread available), which is the case here;
        # indeed, with a single-threaded executor, callbacks are still processed one at a time.
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface for joint-space goals.
        self.moveit2withTorso = MoveIt2(
            node = self, # The ROS2 node that MoveIt2 will attach to.
                         # All necessary internal components (action clients, service, clients, TF lookups, publishers/subscribers) are gonna be created under this node.
            joint_names = JOINT_NAMES_WITH_TORSO, # Ordered list of joint names belonging to the planning group.
                                                  # MoveIt2 expects these names to match the robot’s URDF/SRDF configuration.
            base_link_name = BASE_LINK_NAME, # The root frame of the kinematic chain / manipulator.
                                             # For TIAGO, this is typically "base_link".
                                             # All poses are gonna be interpreted relative to this frame.
            end_effector_name = END_EFFECTOR_NAME, # The frame at the tip of the kinematic chain / manipulator.
                                                   # (e.g., the wrist or tool frame).
                                                   # For TIAGO, this is typically "arm_tool_link".
            group_name = GROUP_NAME_WITH_TORSO, # The MoveIt2 planning group name used for joint-space goals.
                                                # Must match one of the groups defined in the SRDF file, such as "arm_torso" for TIAGO.
            callback_group = self.callback_group, # Callback group controlling how ROS callbacks (action feedback, service responses, timers, TF) are gonna be executed.
                                                  # Again, the ReentrantCallbackGroup allows multiple callbacks to run in parallel and is required for MoveIt2 to work correctly.
        )
        self.moveit2withTorso.planner_id = "RRTConnectkConfigDefault" # Algorithm used for planning the motion: bidirectional RRT-Connect (Rapidly-exploring Random Tree).
        self.moveit2withTorso.planning_time = 4.2 # Maximum time allowed for planning a motion (in seconds). If the planner cannot find a solution within this time, it will return a failure.

        # Create MoveIt 2 interface for Cartesian pose goals.
        self.moveit2ArmOnly = MoveIt2(
            node = self,
            joint_names = ARM_ONLY_JOINT_NAMES,
            base_link_name = BASE_LINK_NAME,
            end_effector_name = END_EFFECTOR_NAME,
            group_name = ARM_ONLY_GROUP_NAME,
            callback_group = self.callback_group,
        )
        self.moveit2ArmOnly.planner_id =  self.moveit2withTorso.planner_id
        self.moveit2ArmOnly.planning_time = self.moveit2withTorso.planning_time
        
        self.get_logger().info(f"MoveIt2 interfaces successfully created. Planner set to: {self.moveit2ArmOnly.planner_id}, planning time set to: {self.moveit2ArmOnly.planning_time:.2f} seconds.")

        # Initializing the Action Server for controlling the Tiago's arm that this Node is gonna expose
        self.actionServer = ActionServer(
            self,
            TiagoArm,
            params.tiagoArmActionName,
            execute_callback = self.actionServerExecuteCallback,
            callback_group = self.callback_group,
        )
        self.get_logger().info(f"TiagoArm Action Server is ready on action name '{params.tiagoArmActionName}'.")

    def actionServerExecuteCallback(self, goal_handle):

        self.get_logger().info("Received new TiagoArm goal.")
        feedbackMessage = TiagoArm.Feedback() # The feedback message that is gonna be sent to the client to inform about the current state of the execution of the goal
        result = TiagoArm.Result() # The result message that is gonna be sent to the client when the goal is completed (successfully or not), to inform about the final outcome of the execution of the goal

        try:
            jointPositions = list(goal_handle.request.joint_positions) # To be used ONLY for goals in the joint space
            positionObj = list(goal_handle.request.position_obj)  # To be used ONLY for goals in the Cartesian space
            quatXYZWObj = list(goal_handle.request.quat_xyzw_obj) # To be used ONLY for goals in the Cartesian space

            hasJointGoal = len(jointPositions) > 0
            hasPoseGoal = len(positionObj) > 0 or len(quatXYZWObj) > 0

            if hasJointGoal and hasPoseGoal:
                msg = "Invalid TiagoArm goal: provide either 'joint_positions' OR 'position_obj and quat_xyzw_obj', NOT both."
                self.get_logger().error(msg)
                feedbackMessage.current_state = "invalid_goal"
                goal_handle.publish_feedback(feedbackMessage)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result

            if not hasJointGoal and not hasPoseGoal:
                msg = "Invalid TiagoArm goal: no target specified at all. Provide either 'joint_positions' or 'position_obj and quat_xyzw_obj'."
                self.get_logger().error(msg)
                feedbackMessage.current_state = "invalid_goal"
                goal_handle.publish_feedback(feedbackMessage)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result

            activeMoveIt2 = None
            # Manage the case of a goal in the joint space
            if hasJointGoal:
                activeMoveIt2 = self.moveit2withTorso
                if len(jointPositions) != len(JOINT_NAMES_WITH_TORSO):
                    msg = f"Invalid TiagoArm joint goal: expected {len(JOINT_NAMES_WITH_TORSO)} joint values, received {len(jointPositions)}."
                    self.get_logger().error(msg)
                    feedbackMessage.current_state = "invalid_goal"
                    goal_handle.publish_feedback(feedbackMessage)
                    goal_handle.abort()
                    result.success = False
                    result.message = msg
                    return result
                self.get_logger().info(
                    f"Planning & waiting for execution by MoveIt2 of the motion to the specified joint configuration using group '{GROUP_NAME_WITH_TORSO}': {jointPositions}"
                )
                feedbackMessage.current_state = "planning"
                goal_handle.publish_feedback(feedbackMessage)
                self.moveit2withTorso.move_to_configuration(jointPositions)
            else:
                # Manage the case of a goal in the Cartesian space
                activeMoveIt2 = self.moveit2ArmOnly
                if len(positionObj) != 3 or len(quatXYZWObj) != 4:
                    msg = f"Invalid TiagoArm pose goal: expected 'position_obj' length 3 and 'quat_xyzw_obj' length 4, received {len(positionObj)} and {len(quatXYZWObj)}."
                    self.get_logger().error(msg)
                    feedbackMessage.current_state = "invalid_goal"
                    goal_handle.publish_feedback(feedbackMessage)
                    goal_handle.abort()
                    result.success = False
                    result.message = msg
                    return result
                quaternionNorm = math.sqrt(sum(q * q for q in quatXYZWObj))
                if quaternionNorm <= 1e-9:
                    msg = "Invalid TiagoArm pose goal: 'quat_xyzw_obj' has zero or near-zero norm."
                    self.get_logger().error(msg)
                    feedbackMessage.current_state = "invalid_goal"
                    goal_handle.publish_feedback(feedbackMessage)
                    goal_handle.abort()
                    result.success = False
                    result.message = msg
                    return result
                quatXYZWObj = [float(q / quaternionNorm) for q in quatXYZWObj]
                positionObj = [float(p) for p in positionObj]

                self.get_logger().info(
                    "Planning & waiting for execution by MoveIt2 of the motion to the specified end-effector pose "
                    f"using group '{ARM_ONLY_GROUP_NAME}': position={positionObj}, quat_xyzw={quatXYZWObj}"
                )
                feedbackMessage.current_state = "planning"
                goal_handle.publish_feedback(feedbackMessage)
                activeMoveIt2.move_to_pose(position=positionObj, quat_xyzw=quatXYZWObj)
            # Planning completed, now executing the motion and waiting for its completion
            feedbackMessage.current_state = "executing"
            goal_handle.publish_feedback(feedbackMessage)
            executionResult = activeMoveIt2.wait_until_executed()
            # Managing the result of the motion execution
            if executionResult is False:
                msg = "Motion execution failed."
                self.get_logger().error(msg)
                feedbackMessage.current_state = "failed"
                goal_handle.publish_feedback(feedbackMessage)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result
            else:
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
