# Python dependencies
import math
import py_trees
import time

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# AUV dependencies
from auv_msgs.action import AUVNavigate

# Planner dependencies
from .action_status_enum import ActionStatus



class BasicActionBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a template behaviour used to create others.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(
            self, 
            name = "ActionBehaviour",
            goal: AUVNavigate = None,
            ) -> None:
            """
            Initializes the node and blackboard client for this behaviour.

            Inputs: str                         : name - the name of the behaviour 
                    auv_msgs.action.AUVNavigate : goal - the goal to send to the action client

            Outputs: None
            """   
            super().__init__(name)
            self.name = name
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.goal = goal
            
            # Initialize fields needed to keep track of goal
            self.sent_goal = False
            self.action_status = ActionStatus.NOT_SENT # Either succeeded, pending, failed or not sent
            self.is_waiting_for_result = False
            
        def setup(self, **kwargs) -> None:
            """
            Description: Sets up keys on the blackboard that this behaviour will use.
            """
            self.node = kwargs['node']
            self.navigation_client = kwargs['shared_nav_client']
            
            # Ensure the action server is ready
            self.navigation_client.client_wait_for_server(timeout_sec=5.0) 

            self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        
        def initialise(self) -> None:
            """
            Description: Called every time this behavior transitions is not RUNNING. The function is used
            to reset the mission status to NOT_SENT whenever the Behaviour is not RUNNING.

            Inputs: None

            Outputs: None
            """
            # Reset field needed to keep track of goal
            self.action_status = ActionStatus.NOT_SENT

                
        def update(self) -> py_trees.common.Status:
            """
            Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

            Ibputs: None

            Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                        py_trees.common.Status.FAILURE if it failed, or 
                        py_trees.common.Status.RUNNING if it is still running.
                
            """
            # Check if pose is properly registered on the blackboard as controls requires
            # a pose value. Block execution if the AUV poses are not published yet
            if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
                self.node.get_logger().info(f"[{self.name}] Waiting for sensor pose data...", throttle_duration_sec=2.0)
                return py_trees.common.Status.RUNNING
                
            # Check for failure condition from the async callbacks (goal response and goal result)
            if self.action_status is ActionStatus.FAILED:
                self.node.get_logger().error(f"[{self.name}] Action failed midway.")
                return py_trees.common.Status.FAILURE
                
            # Completion check
            if self.action_status is ActionStatus.SUCCEEDED:
                self.node.get_logger().info(f"[{self.name}] Completed goal.")
                return py_trees.common.Status.SUCCESS

            # Block loop if currently navigating to a waypoint
            if self.action_status is ActionStatus.PENDING:
                return py_trees.common.Status.RUNNING
                
            # Send the goal if no goals are ongoing and set the mission status to pending
            self.node.get_logger().info(f"[{self.name}] Sent goal.")
            self.navigation_client.send_navigation_goal(self.goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
            self.action_status = ActionStatus.PENDING
            return py_trees.common.Status.RUNNING
        
        def on_server_goal_response(self, goal_response: bool) -> None:
            """
            Description: This function provides customized responses to the 
            action server's decision on accepting the goal or not. In this case,
            the custom implementation updates the status of the mission. Pass this
            function as an input to the navigation_client.send_navigation_goal

            Inputs: goal_response: str, The client will call this function with true upon acceptance, and false upon rejection from the server

            Outputs: None
            """
            if not goal_response:
                self.action_status = ActionStatus.FAILED

        def on_server_goal_result(self, goal_success: bool) -> None:
            """
            Description: This function provides customized logic to be executed when
            the goal is finished. In this case, the custom implementation updates the status of the mission
            depending on whether or not the goal was successful or failed. Pass this function as an input 
            to the navigation_client.send_navigation_goal

            Inputs: goal_success: str, The client will call this function with true upon success, and false upon failure

            Outputs: None
            """
            if goal_success:
                self.action_status = ActionStatus.SUCCEEDED
            else:
                self.action_status = ActionStatus.FAILED



class BasicTriggerServiceBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a template behaviour used to create trigger service behaviours.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(
            self, 
            name = "ActionBehaviour",
            service_name = None,
            timeout_sec: float = 10.0,
            max_retries: int = 3
            ) -> None:
            """
            Initializes the node and blackboard client for this behaviour.

            Inputs: str                         : name - the name of the behaviour 
                    str                         : service_name - the name of the service to call
                    float                       : timeout_sec - max seconds to wait for a response (default: 10.0)
                    int                         : max_retries - max number of attempts before permanent failure (default: 3)

            Outputs: None
            """   
            super().__init__(name)
            self.name = name
            self.service_name = service_name
            self.timeout_sec = timeout_sec
            self.max_retries = max_retries
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.sent_service_request = False
            self.future = None
            self.request_time = None
            self.attempt_count = 0
            self._exhausted_logged = False

        def setup(self, **kwargs) -> None:
            """
            Description: Sets up keys on the blackboard that this behaviour will use.
            """
            self.node = kwargs['node']
            self.service_client = self.node.create_client(Trigger, self.service_name)
            
            # Check if the service is available at startup
            if not self.service_client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().warn(f"[{self.name}] Service '{self.service_name}' not yet available. Will retry when triggered.")


        def initialise(self) -> None:
            """
            Description: Called every time this behavior transitions is not RUNNING. The function is used
            to reset the mission status to NOT_SENT whenever the Behaviour is not RUNNING.

            Inputs: None

            Outputs: None
            """
            # Reset field needed to keep track of goal
            self.sent_service_request = False
            self.future = None
            self.request_time = None

        def update(self) -> py_trees.common.Status:
            """
            Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

            Ibputs: None

            Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                        py_trees.common.Status.FAILURE if it failed, or 
                        py_trees.common.Status.RUNNING if it is still running.
                
            """
            # Check if max retries exhausted
            if self.attempt_count >= self.max_retries and not self.sent_service_request:
                if not self._exhausted_logged:
                    self.node.get_logger().error(f"[{self.name}] Max retries ({self.max_retries}) exhausted. Giving up.")
                    self._exhausted_logged = True
                return py_trees.common.Status.FAILURE

            # Send the request asynchronously
            if not self.sent_service_request:
                self.attempt_count += 1
                self.node.get_logger().info(f"[{self.name}] Sending service request (attempt {self.attempt_count}/{self.max_retries}).")
                request = Trigger.Request()
                self.future = self.service_client.call_async(request)
                self.sent_service_request = True
                self.request_time = self.node.get_clock().now()
                return py_trees.common.Status.RUNNING
            
            # Still waiting for response
            if not self.future.done():
                # Check for timeout
                elapsed = (self.node.get_clock().now() - self.request_time).nanoseconds / 1e9
                if elapsed >= self.timeout_sec:
                    self.node.get_logger().error(f"[{self.name}] Service call timed out after {self.timeout_sec:.1f}s")
                    self.future.cancel()
                    # Reset to allow retry on the next tick (stays RUNNING)
                    self.sent_service_request = False
                    return py_trees.common.Status.RUNNING
                return py_trees.common.Status.RUNNING
            
            # Check if the service call itself failed (e.g. middleware error)
            if self.future.exception() is not None:
                self.node.get_logger().error(f"[{self.name}] Service call failed: {self.future.exception()}")
                # Reset to allow retry on the next tick (stays RUNNING)
                self.sent_service_request = False
                return py_trees.common.Status.RUNNING
            
            # Verify if service is successful or not
            response = self.future.result()
            if response.success:
                self.node.get_logger().info(f"[{self.name}] Service succeeded.")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().info(f"[{self.name}] Service failed: {response.message}")
                return py_trees.common.Status.FAILURE



class TimerBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a timer to be implemented. The timer is dictated by measuring a time.time()
        interval and checking if it is above a desired threshold upon a new tick's arrival. This not fully accurate  
        method is acceptable since the current scope of this TimerBehaviour is to allow time to untether Douglas

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(self, timer_duration: float, name="sensorsLeaf") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: str                : name - the name of the behaviour 
                        float              : timer_duration - the desired duration of the timer

                Outputs: None
                """   
                super().__init__(name)
                self.timer_duration = timer_duration
                self.start_time = 0.0
                self.timer_started = False

        def setup(self, **kwargs) -> None:
                self.node = kwargs['node']
        
        def initialise(self) -> None:
                """
                Whenever the Behaviour is not in RUNNING state, reset the timer to not started

                Inputs: None

                Outputs: None
                """
                self.timer_started = False

        def update(self) -> py_trees.common.Status:
                """
                Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

                Ibputs: None

                Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                         py_trees.common.Status.FAILURE if it failed, or 
                         py_trees.common.Status.RUNNING if it is still running.
                 
                """
                if not self.timer_started:
                        # Set the initial time at start of time counting
                        self.start_time = self.node.get_clock().now().nanoseconds / 1e9
                        self.timer_started = True
                        return py_trees.common.Status.RUNNING
                
                current_time = self.node.get_clock().now().nanoseconds / 1e9
                # Upon each tick, check if interval of time has been passed
                self.node.get_logger().info(f"Time passed: {(current_time - self.start_time)}")
                if (current_time - self.start_time) > float(self.timer_duration):
                        return py_trees.common.Status.SUCCESS
                
                return py_trees.common.Status.RUNNING
