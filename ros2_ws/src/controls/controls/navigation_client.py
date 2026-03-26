#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from auv_msgs.action import AUVNavigate
from . import goal_helpers
from std_msgs.msg import String

class NavigationClient(Node):
        """ A client for interacting with the navigation action server. """

        def __init__(self, name="navigation_client", debug=False) -> None:
                """ Initializes the navigation client node. """
                super().__init__(name)
                self._action_client = ActionClient(self, AUVNavigate, '/motion/navigate')
                self.current_goal_handle = None # Store the goal handle of the currently active goal, if any, to allow for cancellation when a new goal is sent.)
                self.debug = debug
                self.current_caller_publisher = self.create_publisher(String, '/controls/client/caller', 10)

        def send_navigation_goal(self, 
                goal_msg: AUVNavigate, 
                caller_name: String, 
                custom_goal_response=None, 
                custom_goal_result=None) -> None:
                """ Send a navigation goal to the action server. If active goal in progress, cancel it before sending the new one.
                
                Args: goal_msg (auv_msgs.action.AUVNavigate): The navigation goal message
                      caller_name (str)                     : The Behaviour sending the goal, this is for debug purposes
                      goal_response_callback                : The goal response callback, can be customized
                Outputs: None, but will log the result and set up the result callback."""

                # Check if there is an active goal and cancel it before sending a new one
                if self.current_goal_handle is not None:
                        if self.debug: self.get_logger().info("Cancelling current navigation goal before sending a new one.")
                        cancel_future = self.current_goal_handle.cancel_goal_async()
                        cancel_future.add_done_callback(self.cancel_done_callback)

                # Publish the current caller
                caller_info_msg = String()
                caller_info_msg.data = caller_name
                self.current_caller_publisher.publish(caller_info_msg)
                
                # Create a new goal 
                goal_to_send = AUVNavigate.Goal()

                # Match the goal to send with the goal message type
                goal_to_send.target_pose = goal_msg.target_pose
                goal_to_send.do_x = goal_msg.do_x
                goal_to_send.do_y = goal_msg.do_y
                goal_to_send.do_z = goal_msg.do_z
                goal_to_send.do_yaw = goal_msg.do_yaw
                goal_to_send.is_relative = goal_msg.is_relative
                goal_to_send.is_local_frame = goal_msg.is_local_frame
                goal_to_send.position_tolerance = goal_msg.position_tolerance
                goal_to_send.yaw_tolerance = goal_msg.yaw_tolerance
                goal_to_send.hold_time = goal_msg.hold_time
                goal_to_send.timeout = goal_msg.timeout

                self._send_goal_future = self._action_client.send_goal_async(goal_to_send, feedback_callback=self.feedback_callback)
                # Set callback upon goal response, use lambda function to allow for extra parameters
                self._send_goal_future.add_done_callback(lambda future: 
                        self.goal_response_callback(future, custom_goal_response, custom_goal_result))

        def goal_response_callback(self, future: rclpy.task.Future, custom_goal_response, custom_goal_result):
                """ Callback for handling the response from the action server after sending a goal.
                Invoked when the action server accepts or rejects the goal, which is asynchronous. 
                
                Args: future (rclpy.task.Future): The future object representing the asynchronous goal response.
                Outputs: None, but will log the result and set up the result callback if the goal is accepted."""
                
                goal_handle = future.result()
                if custom_goal_response != None:
                        custom_goal_response(goal_response=goal_handle.accepted)
                # Check if rejected goal
                if not goal_handle.accepted:
                        if self.debug: self.get_logger().info("Navigation goal rejected")
                        return

                if self.debug: self.get_logger().info("Navigation goal accepted")
                self.current_goal_handle = goal_handle # Store the goal handle for later use (e.g., to keep track on if client is processing a goal)
                self._get_result_future = goal_handle.get_result_async()

                # Set callback upon result, use lambda function to allow for extra parameters
                self._get_result_future.add_done_callback(lambda future:
                        self.get_result_callback(future, custom_goal_result))
        
        def get_result_callback(self, future: rclpy.task.Future, custom_goal_result) -> None:
                """Callback for handling the result from the action server after the goal is completed. Sets the 
                current goal handle to None since the goal is completed, and logs the result. Invoked when the action server accepts or rejects the goal, 
                which is asynchronous. 
                
                Args: future (rclpy.task.Future): The future object representing the asynchronous result response.
                Outputs: None, but will log the result."""
                
                result = future.result()
                if self.debug: self.get_logger().info(f"Navigation result received: {result}")

                # Perform the custom goal result, if provided by the user
                if custom_goal_result != None:
                        custom_goal_result(goal_success=result)
                # Clear the current goal handle since the goal is completed
                self.current_goal_handle = None
        
        def feedback_callback(self, feedback_msg: AUVNavigate.Feedback) -> None:
                """Callback for handling the feedback of current goal progress from the action server while the goal is processed.  
                Invoked at the frequency set by the action server for sending feedback, which is asynchronous.
                
                Args: feedback_msg (AUVNavigate.Feedback): The feedback message from the action server.
                Outputs: None, but will log the result."""
                
                feedback = feedback_msg.feedback
                if self.debug: self.get_logger().info(f"Navigation feedback: {feedback}")

        def cancel_done_callback(self, future: rclpy.task.Future) -> None:
                """Callback for handling the result of a goal cancellation request. Logs the result of the cancellation.
                Invoked when the action server processes the cancellation request, which is asynchronous.
                
                Args: future (rclpy.task.Future): The future object representing the asynchronous cancellation response.
                Outputs: None, but will log the result."""
                
                cancel_response = future.result()
                if self.debug: self.get_logger().info(f"Navigation goal cancellation response: {cancel_response}")

        def client_wait_for_server(self, timeout_sec):
                self._action_client.wait_for_server(timeout_sec)

        def reset_action_client(self) -> None:
                """ Resets the action client by canceling active goal and clearing the current goal handle.
                Also sets the on restult callback to the default get_result_callback method. 

                Args: None
                Outputs: None, but will log the reset action. """

                if self.current_goal_handle is not None:
                        if self.debug: self.get_logger().info("Reset action client: Cancelling active navigation goal.")
                        cancel_future = self.current_goal_handle.cancel_goal_async()
                        cancel_future.add_done_callback(self.cancel_done_callback)
                        self.current_goal_handle = None
                if self.debug: self.get_logger().info("Navigation action client has been reset.")