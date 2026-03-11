#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from auv_msgs.action import AUVNavigate

class NavigationClient(Node):
        """ A client for interacting with the navigation action server. """

        def __init__(self, name="navigation_client"):
                """ Initializes the navigation client node. """
                super().__init__(name)
                self._action_client = ActionClient(self, AUVNavigate, 'navigate_to_pose')
                self.current_goal_handle = None # Store the goal handle of the currently active goal, if any, to allow for cancellation when a new goal is sent.)

        def send_navigation_goal(self, goal_msg: AUVNavigate) -> None:
                """ Send a navigation goal to the action server. If active goal in progress, cancel it before sending the new one.
                
                Args: goal_msg (auv_msgs.action.AUVNavigate): The navigation goal message
                Outputs: None, but will log the result and set up the result callback."""
                
                # Check if there is an active goal and cancel it before sending a new one
                if self.current_goal_handle is not None:
                        self.get_logger().info("Cancelling current navigation goal before sending a new one.")
                        cancel_future = self.current_goal_handle.cancel_goal_async()
                        cancel_future.add_done_callback(self.cancel_done_callback)
                
                # Create a new goal 
                goal_to_send = AUVNavigate.Goal()

                # Match the goal to send with the goal message type
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

                self._action_client.wait_for_server()
                self._send_goal_future = self._action_client.send_goal_async(goal_to_send, feedback_callback=self.feedback_callback)
                self._send_goal_future.add_done_callback(self.goal_response_callback)

        def goal_response_callback(self, future: rclpy.task.Future):
                """ Callback for handling the response from the action server after sending a goal.
                Invoked when the action server accepts or rejects the goal, which is asynchronous. 
                
                Args: future (rclpy.task.Future): The future object representing the asynchronous goal response.
                Outputs: None, but will log the result and set up the result callback if the goal is accepted."""
                
                goal_handle = future.result()
                
                if not goal_handle.accepted:
                        self.get_logger().info("Navigation goal rejected")
                        return
                self.get_logger().info("Navigation goal accepted")
                self.current_goal_handle = goal_handle # Store the goal handle for later use (e.g., to a goal when a new one is given)
                self._get_result_future = goal_handle.get_result_async()
                self._get_result_future.add_done_callback(self.get_result_callback)
        
        def get_result_callback(self, future: rclpy.task.Future) -> None:
                """Callback for handling the result from the action server after the goal is completed. Sets the 
                current goal handle to None since the goal is completed, and logs the result. Invoked when the action server accepts or rejects the goal, 
                which is asynchronous. 
                
                Args: future (rclpy.task.Future): The future object representing the asynchronous result response.
                Outputs: None, but will log the result."""
                
                result = future.result()
                self.get_logger().info(f"Navigation result received: {result}")
                self.current_goal_handle = None # Clear the current goal handle since the goal is completed
        
        def feedback_callback(self, feedback_msg: AUVNavigate.Feedback) -> None:
                """Callback for handling the feedback of current goal progress from the action server while the goal is processed.  
                Invoked at the frequency set by the action server for sending feedback, which is asynchronous.
                
                Args: feedback_msg (AUVNavigate.Feedback): The feedback message from the action server.
                Outputs: None, but will log the result."""
                
                feedback = feedback_msg.feedback
                self.get_logger().info(f"Navigation feedback: {feedback}")