# This Python file defines a ROS2 Humble Node which exposes an Action Server to be used to control the Tiago's gripper.

from rclpy.node import Node
from rclpy.action import ActionServer
from tiago_exam_interfaces.action import TiagoGripper
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import GripperInterface
from linkattacher_msgs.srv import AttachLink, DetachLink
import rclpy
import time
from threading import Thread
from . import nodesParameters

# TODO: manage the possibility (unwanted, but possible) of receiving a new goal while one is already being executed

# Gripper parameters
GRIPPER_JOINT_NAMES = [
    "gripper_left_finger_joint",
    "gripper_right_finger_joint",
]

OPEN_GRIPPER_POSITIONS = [0.04, 0.04]  # One value for each Tiago gripper joint, 4cm each.
CLOSED_GRIPPER_POSITIONS = [0.032, 0.032]
FULL_CLOSED_GRIPPER_POSITION = 0.0

# Name of the MoveIt planning group that contains the gripper joints.
GRIPPER_GROUP_NAME = "gripper"

# ROS2 action used to command the gripper fingers.
GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"

# Gazebo LinkAttacher default parameters.
LINK_ATTACHER_ATTACH_SERVICE_NAME = "/ATTACHLINK"
LINK_ATTACHER_DETACH_SERVICE_NAME = "/DETACHLINK"
LINK_ATTACHER_TIAGO_MODEL_NAME = "tiago"
LINK_ATTACHER_GRIPPER_LINK_NAME = "gripper_left_finger_link"
LINK_ATTACHER_WAIT_TIMEOUT = 3.0
LINK_ATTACHER_CALL_TIMEOUT = 5.0


class TiagoGripperNode(Node):

    def __init__(self):

        super().__init__('tiago_gripper_node')
        self.get_logger().info("Starting tiago_gripper_node initialization...")

        # ReentrantCallbackGroup allows callbacks associated with the same node/group to be executed without strict mutual exclusion, which is useful here because
        # indeed within this node the action server and the internal ROS interfaces used by GripperInterface may need to make progress concurrently.
        # This choice only becomes truly meaningful when the node is spun by a MultiThreadedExecutor (i.e. with more than one worker thread);
        # indeed, with a single-threaded executor, callbacks are still processed one at a time.
        self.callback_group = ReentrantCallbackGroup()

        # Create gripper interface. This does not require a full MoveIt2 arm interface: it commands the gripper trajectory controller.
        self.gripper = GripperInterface(
            node = self,
            gripper_joint_names = GRIPPER_JOINT_NAMES,
            open_gripper_joint_positions = OPEN_GRIPPER_POSITIONS,
            closed_gripper_joint_positions = CLOSED_GRIPPER_POSITIONS,
            gripper_group_name = GRIPPER_GROUP_NAME,
            callback_group = self.callback_group,
            gripper_command_action_name = GRIPPER_COMMAND_ACTION_NAME,
        )
        self.get_logger().info("Gripper interface successfully created.")

        self.attachLinkClient = self.create_client(
            AttachLink,
            LINK_ATTACHER_ATTACH_SERVICE_NAME,
            callback_group = self.callback_group,
        )
        self.detachLinkClient = self.create_client(
            DetachLink,
            LINK_ATTACHER_DETACH_SERVICE_NAME,
            callback_group = self.callback_group,
        )
        self.attachedObjectModelName = ""
        self.attachedObjectLinkName = ""

        # Initializing the Action Server for controlling the Tiago's gripper that this Node is gonna expose.
        self.actionServer = ActionServer(
            self,
            TiagoGripper,
            nodesParameters.tiagoGripperActionName,
            execute_callback = self.actionServerExecuteCallback,
            callback_group = self.callback_group,
        )
        self.get_logger().info(f"TiagoGripper Action Server is ready on action name '{nodesParameters.tiagoGripperActionName}'.")

    def wait_for_future_without_spinning_this_node(self, future, timeout_sec: float) -> bool:
        startTime = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - startTime > float(timeout_sec):
                return False
            time.sleep(0.02)
        return future.done()

    def _call_link_attacher_service(self, client, requestType, service_name: str, object_model_name: str, object_link_name: str) -> bool:
        if object_model_name is None or str(object_model_name).strip() == "":
            self.get_logger().warn(f"Skipping {service_name}: empty object_model_name.")
            return False
        if object_link_name is None or str(object_link_name).strip() == "":
            self.get_logger().warn(f"Skipping {service_name}: empty object_link_name.")
            return False

        self.get_logger().info(
            f"Waiting for LinkAttacher service '{service_name}' "
            f"to manage {LINK_ATTACHER_TIAGO_MODEL_NAME}/{LINK_ATTACHER_GRIPPER_LINK_NAME} "
            f"<-> {str(object_model_name).strip()}/{str(object_link_name).strip()}..."
        )
        if not client.wait_for_service(timeout_sec = LINK_ATTACHER_WAIT_TIMEOUT):
            self.get_logger().error(f"LinkAttacher service '{service_name}' is not available.")
            return False

        req = requestType()
        req.model1_name = LINK_ATTACHER_TIAGO_MODEL_NAME
        req.link1_name = LINK_ATTACHER_GRIPPER_LINK_NAME
        req.model2_name = str(object_model_name).strip()
        req.link2_name = str(object_link_name).strip()

        self.get_logger().info(
            f"Calling LinkAttacher service '{service_name}': "
            f"{req.model1_name}/{req.link1_name} <-> {req.model2_name}/{req.link2_name}."
        )
        future = client.call_async(req)

        if not self.wait_for_future_without_spinning_this_node(future, LINK_ATTACHER_CALL_TIMEOUT):
            self.get_logger().error(f"LinkAttacher service '{service_name}' call timed out.")
            return False

        try:
            response = future.result()
            if hasattr(response, 'success') and not bool(response.success):
                message = response.message if hasattr(response, 'message') else "no message"
                self.get_logger().error(f"LinkAttacher service '{service_name}' returned failure: {message}")
                return False

            message = response.message if hasattr(response, 'message') else "no message"
            self.get_logger().info(f"LinkAttacher service '{service_name}' completed successfully: {message}")
            return True
        except Exception as exc:
            self.get_logger().error(f"LinkAttacher service '{service_name}' call failed: {exc}")
            return False

    def attach_object(self, object_model_name: str, object_link_name: str) -> bool:
        success = self._call_link_attacher_service(
            self.attachLinkClient,
            AttachLink.Request,
            LINK_ATTACHER_ATTACH_SERVICE_NAME,
            object_model_name,
            object_link_name,
        )
        if success:
            self.attachedObjectModelName = str(object_model_name).strip()
            self.attachedObjectLinkName = str(object_link_name).strip()
        return success

    def detach_object(self, object_model_name: str = "", object_link_name: str = "") -> bool:
        modelName = str(object_model_name).strip() if object_model_name else self.attachedObjectModelName
        linkName = str(object_link_name).strip() if object_link_name else self.attachedObjectLinkName

        if modelName == "" or linkName == "":
            self.get_logger().info("No attached object remembered. Skipping LinkAttacher detach.")
            return True

        success = self._call_link_attacher_service(
            self.detachLinkClient,
            DetachLink.Request,
            LINK_ATTACHER_DETACH_SERVICE_NAME,
            modelName,
            linkName,
        )
        if success and modelName == self.attachedObjectModelName and linkName == self.attachedObjectLinkName:
            self.attachedObjectModelName = ""
            self.attachedObjectLinkName = ""
        return success

    def actionServerExecuteCallback(self, goal_handle):

        self.get_logger().info("Received new TiagoGripper goal.")
        feedbackMessage = TiagoGripper.Feedback()
        result = TiagoGripper.Result()

        try:
            openCommand = bool(goal_handle.request.open)
            commandLabel = "open" if openCommand else "close"
            objectModelName = str(goal_handle.request.object_model_name).strip()
            objectLinkName = str(goal_handle.request.object_link_name).strip()

            if openCommand:
                feedbackMessage.current_state = "detaching"
                goal_handle.publish_feedback(feedbackMessage)
                if not self.detach_object(objectModelName, objectLinkName):
                    msg = "Gripper open aborted because LinkAttacher detach failed."
                    self.get_logger().error(msg)
                    feedbackMessage.current_state = "failed"
                    goal_handle.publish_feedback(feedbackMessage)
                    goal_handle.abort()
                    result.success = False
                    result.message = msg
                    return result

            self.get_logger().info(f"Planning & waiting for execution of gripper command: {commandLabel}.")
            feedbackMessage.current_state = "planning"
            goal_handle.publish_feedback(feedbackMessage)

            if openCommand:
                self.gripper.open()
            else:
                if objectModelName != "" and objectLinkName != "":
                    self.gripper.close()
                else:
                    self.get_logger().info(
                        "Gripper close requested without complete LinkAttacher target. Using full closed gripper position."
                    )
                    self.gripper.move_to_position(float(FULL_CLOSED_GRIPPER_POSITION))

            feedbackMessage.current_state = "executing"
            goal_handle.publish_feedback(feedbackMessage)
            execution_result = self.gripper.wait_until_executed()

            if execution_result is False:
                msg = f"Gripper {commandLabel} execution failed."
                self.get_logger().error(msg)
                feedbackMessage.current_state = "failed"
                goal_handle.publish_feedback(feedbackMessage)
                goal_handle.abort()
                result.success = False
                result.message = msg
                return result

            if not openCommand:
                if objectModelName != "" and objectLinkName != "":
                    feedbackMessage.current_state = "attaching"
                    goal_handle.publish_feedback(feedbackMessage)
                    if not self.attach_object(objectModelName, objectLinkName):
                        msg = "Gripper close completed, but LinkAttacher attach failed."
                        self.get_logger().error(msg)
                        feedbackMessage.current_state = "failed"
                        goal_handle.publish_feedback(feedbackMessage)
                        goal_handle.abort()
                        result.success = False
                        result.message = msg
                        return result
                else:
                    self.get_logger().info(
                        "Gripper close requested without complete LinkAttacher target. Skipping attach."
                    )

            feedbackMessage.current_state = "completed"
            goal_handle.publish_feedback(feedbackMessage)
            goal_handle.succeed()
            result.success = True
            result.message = f"Gripper {commandLabel} completed successfully."
            self.get_logger().info(result.message)
            return result

        except Exception as exc:
            msg = f"Exception while executing TiagoGripper action: {exc}"
            self.get_logger().error(msg)
            feedbackMessage.current_state = "exception"
            goal_handle.publish_feedback(feedbackMessage)
            goal_handle.abort()
            result.success = False
            result.message = msg
            return result


def main(args=None):

    rclpy.init(args=args)
    node = TiagoGripperNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads = 2) # Create a ROS2 executor that can run callbacks in parallel using 2 worker threads.
                                                                      # This is required because GripperInterface uses callbacks/action clients internally.
    executor.add_node(node) # Register this very TiagoGripperNode with the executor so that all its subscriptions,
                            # timers, services and action clients become active and start receiving events and callbacks.
    spin_thread = Thread(target = executor.spin, daemon = True, args = ()) # Create a new Python thread that will run executor.spin().
                                                                           # Note that executor.spin() is a *blocking* call:
                                                                           # it enters an infinite loop and continuously handles ROS2 callbacks.
                                                                           # Running spin() in a separate thread allows the main thread to continue executing the remaining code.
    spin_thread.start() # Start the executor thread.
                        # From this point, ROS2 is actively processing callbacks in the background.

    node.get_logger().info("TiagoGripper node now spinning with MultiThreadedExecutor.")

    try:
        spin_thread.join() # Wait for the executor thread to finish (which is basically waiting indefinitely until the node is shut down)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down TiagoGripper node...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__": main()

