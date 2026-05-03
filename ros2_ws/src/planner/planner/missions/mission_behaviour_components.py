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

        def on_server_goal_result(self, goal_success) -> None:
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

        def terminate(self, new_status: py_trees.common.Status):
            """Called if the tree aborts this branch or if it naturally finishes."""
            if new_status == py_trees.common.Status.INVALID:
                if hasattr(self, 'node') and self.node:
                    self.node.get_logger().warn(f"[{self.name}] Aborted branch. Canceling active goal.")
                if hasattr(self, 'navigation_client') and self.navigation_client:
                    self.navigation_client.reset_action_client()


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
            service_name = None
            ) -> None:
            """
            Initializes the node and blackboard client for this behaviour.

            Inputs: str                         : name - the name of the behaviour 
                    str                         : service_name - the name of the service to call

            Outputs: None
            """   
            super().__init__(name)
            self.name = name
            self.service_name = service_name
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.sent_service_request = False
            self.future = None

        def setup(self, **kwargs) -> None:
            """
            Description: Sets up keys on the blackboard that this behaviour will use.
            """
            self.node = kwargs['node']
            self.service_client = self.node.create_client(Trigger, self.service_name)
            # FIND A BETTER WAY TO DO THIS
            
            # Check if the service is available, if not log an error and raise an exception
            #if not self.service_client.wait_for_service(timeout_sec=2.0):
            #    self.node.get_logger().error(f"[{self.name}] Service not available.")
            #    raise RuntimeError(f"Service {self.service_client.srv_name} not available.")


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

        def update(self) -> py_trees.common.Status:
            """
            Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

            Ibputs: None

            Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                        py_trees.common.Status.FAILURE if it failed, or 
                        py_trees.common.Status.RUNNING if it is still running.
                
            """
            # Set the request and send it asynchronously
            if not self.sent_service_request:
                self.node.get_logger().info(f"[{self.name}] Sending service request.")
                request = Trigger.Request()
                self.future = self.service_client.call_async(request)
                self.sent_service_request = True
                return py_trees.common.Status.RUNNING
            
            # Verify if the service response has been received
            if not self.future.done():
                return py_trees.common.Status.RUNNING
            
            # Verify if service is successful or not
            try:
                response = self.future.result()
                if response.success:
                    self.node.get_logger().info(f"[{self.name}] Service succeeded.")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.node.get_logger().info(f"[{self.name}] Service failed.")
                    return py_trees.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(f"[{self.name}] Service call failed with exception: {e}")
                return py_trees.common.Status.FAILURE


class MissionChoiceCheckBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents the Mission Choice checker behavior used to provide
        mission selections.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(self, choice: int, name="MissionChoiceCheckBehaviour") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: rclpy.node.Node    : node - the ROS2 node to use for subscribing to topics 
                        str                : name - the name of the behaviour 

                Outputs: None
                """   
                super().__init__(name)
                self.blackboard = self.attach_blackboard_client(name=self.name)
                self.choice = choice

        def setup(self, **kwargs) -> None:
                """
                Description: Sets up keys on the blackboard that this behaviour will use.
                """
                self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.READ) 
                
        def update(self) -> py_trees.common.Status:
                """
                Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

                Ibputs: None

                Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                         py_trees.common.Status.FAILURE if it failed, or                  
                """
                # Check if the user's choice matches the mission's choice
                if self.choice == self.blackboard.mission_choice:
                        return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.FAILURE
        

class MissionCompleteBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour serves as resetting the user input when a mission is finished.
        When the user input is empty, the user will once again get the choice to choose a mission

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(self, name="MissionCompleteBehaviour") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: str                : name - the name of the behaviour 

                Outputs: None
                """   
                super().__init__(name)
                self.blackboard = self.attach_blackboard_client(name=self.name)

        def setup(self, **kwargs) -> None:
                """
                Description: Sets up keys on the blackboard that this behaviour will use.
                """
                self.node = kwargs['node']
                self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.WRITE)
                self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.READ)

        def update(self) -> py_trees.common.Status:
                """
                Description: This function is called at the end a mission sequence.
                Always returns SUCCESS

                Inputs: None

                Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                 
                """
                self.node.get_logger().info(f"Mission Completed! {self.blackboard.mission_choice}")
                self.blackboard.mission_choice = None
                return py_trees.common.Status.RUNNING
        

class TimerBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a timer to be implemented. The timer is dictated by measuring a time.time()
        interval and checking if it is above a desired threshold upon a new tick's arrival. This not fully accurate  
        method is acceptable since the current scope of this TimerBehaviour is to allow time to untether Douglas

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(self, timer: float, name="sensorsLeaf") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: str                : name - the name of the behaviour 
                        float              : timer - the desired duration of the timer

                Outputs: None
                """   
                super().__init__(name)
                self.timer = timer
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
                if (current_time - self.start_time) > float(self.timer):
                        return py_trees.common.Status.SUCCESS
                
                return py_trees.common.Status.RUNNING
