import geometry_msgs
import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from py_trees.common import Status, Access
from py_trees.blackboard import Client
from auv_msgs.action import AUVNavigate
import math
from controls.goal_helpers import move_robot_centric, rotate_relative, _make_goal

class CustomCallback(py_trees.behaviour.Behaviour):
    def __init__(self, name="CustomCallback", node=None, rotations_segments=5, angle_to_rotate_deg=90, radius_to_rotate_meter=20.0, clockwise=False) -> None:
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.goal_sent = False
        self.navigation_client = None
        self.rotations_segments = rotations_segments
        self.remaining_segments = rotations_segments
        self.rotate_or_translate = 0  # 0 for initial state, 1 for translation in progress, 2 for rotation in progress
        self.goal_status = None
        self.clockwise = clockwise

        # Calculate the rotation and translation per segment based on the total angle to rotate and the number of segments, and store them on the blackboard for use in the custom callback
        self.rotate_per_segment = math.radians(angle_to_rotate_deg) / self.rotations_segments
        self.translation_per_segment = math.sqrt(2 * radius_to_rotate_meter**2 * (1 - math.cos(self.rotate_per_segment))) # This would be calculated based on the desired radius of rotation
        self.forward = math.sin(self.rotate_per_segment/2) * self.translation_per_segment
        self.sway = -math.cos(self.rotate_per_segment/2) * self.translation_per_segment # Currently times -1 due to frame, need to check if its a bug or no
        self.initial_translation_tolerance = 0.4 * self.translation_per_segment
        self.yaw_tolerance = 0.1 * self.rotate_per_segment
        
        # If the rotation is clockwise, invert the angle to rotate and the sway direction
        if self.clockwise:
            self.rotate_per_segment = -self.rotate_per_segment
            self.sway = -self.sway
        

    def setup(self) -> None:
        """
        Description: Sets up keys on the blackboard that this behaviour will use.
        """
        # These blackboard keys will be used by other behaviours whenever they want to cancel this Rotation Behaviour 
        # by writing the goal_sent to false and cancelling the ongoing goal in the navigation client.
        # This methodology will allow this Behaviour to automatically restart all its variables on its own when it is ticked again
        self.blackboard.register_key(key="/navigation_client/rotate_around_point/goal_sent", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/navigation_client/rotate_around_point/goal_sent", access=py_trees.common.Access.READ)

        self.blackboard.register_key(key="/navigation_client/ongoing_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key(key="/navigation_client/client", access=py_trees.common.Access.READ)
        self.navigation_client = self.blackboard.navigation_client.client # Get the navigation client from the blackboard to send goals in the custom callback

        # Initialize the blackboard keys for the goal sent flag and ongoing goal
        self.blackboard.navigation_client.rotate_around_point.goal_sent = False


    def update(self) -> Status:
        self.node.get_logger().info("Custom Callback Behaviour Tick")
        if not self.blackboard.navigation_client.rotate_around_point.goal_sent:
            self.navigation_client.set_custom_result_callback(self.result_callback) # Set the custom callback for handling the result from the action server
            self.blackboard.navigation_client.rotate_around_point.goal_sent = True
            self.remaining_segments = self.rotations_segments # Reset the remaining segments to rotate at the start of the behaviour

            # Send the first goal for the first segment here, 
            self.node.get_logger().info(f"Sending first goal for rotation around point behaviour: {self.sway} {self.forward} segments.")
            self.navigation_client.send_navigation_goal(move_robot_centric(forward=self.forward, sway=self.sway, heave=0.0, dyaw=self.rotate_per_segment, tolerance=self.initial_translation_tolerance, yaw_tolerance=self.yaw_tolerance, hold_time=0.2, timeout=10.0)) # Send goal for translation for the next segment # Send goal for rotation for the next segment
            #self.navigation_client.send_navigation_goal(move_robot_centric(forward=3.0, sway=0.0, heave=0.0, dyaw=0.0, tolerance=0.05, yaw_tolerance=0.1, hold_time=0.2, timeout=3.0))
            return Status.RUNNING

        elif self.blackboard.navigation_client.rotate_around_point.goal_sent and self.remaining_segments == 0 and \
            self.blackboard.navigation_client.ongoing_goal is None: # Check if the goal has been sent, all segments have been completed, and there is no ongoing goal (i.e., the last goal has been completed)
            
            self.blackboard.navigation_client.rotate_around_point.goal_sent = False # Reset the goal sent flag on the blackboard to allow the behaviour to be triggered again in the future
            self.node.get_logger().info("Completed all segments of rotation around point behaviour.")
            return Status.SUCCESS

        elif self.blackboard.navigation_client.rotate_around_point.goal_sent and self.blackboard.navigation_client.ongoing_goal is not None:
            self.node.get_logger().info("Goal already sent, waiting for result...")
            return Status.RUNNING

        elif self.goal_status == "failure": # Check if the goal result was a failure, if so return failure and reset the goal status for the next time the behaviour is triggered
            self.goal_status = None # Reset the goal status for the next time the behaviour is triggered
            self.blackboard.navigation_client.rotate_around_point.goal_sent = False # Reset the goal sent flag on the blackboard to allow the behaviour to be triggered again in the future
            self.navigation_client.reset_action_client() # Reset the navigation action client to clear any potential issues and allow for a fresh start next time the behaviour is triggered
            self.node.get_logger().info("Rotation around point behaviour failed.")

            return Status.FAILURE
        else:
            return Status.RUNNING

    def result_callback(self, future: rclpy.task.Future) -> None:
        result_status = future.result().result.success
        self.node.get_logger().info(f"Custom callback received result: {result_status}")

        if result_status == False:  
            self.goal_status = "failure"

        if self.remaining_segments > 0 and result_status == True: # If there are remaining segments to rotate and the last goal was successful, send the next goal for the next segment
            self.remaining_segments -= 1
            self.translation_tolerance = 0.3 *self.initial_translation_tolerance if self.remaining_segments == 1 else self.initial_translation_tolerance # Decrease the translation tolerance for the last segment to allow for proper settling into the final position
            self.navigation_client.send_navigation_goal(move_robot_centric(forward=self.forward, sway=self.sway, heave=0.0, dyaw=self.rotate_per_segment, tolerance=self.translation_tolerance, yaw_tolerance=self.yaw_tolerance, hold_time=0.2, timeout=10.0)) 