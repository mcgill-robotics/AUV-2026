import geometry_msgs
import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from py_trees.common import Status, Access
from py_trees.blackboard import Client
from rclpy.action import ActionClient
from auv_msgs.action import AUVNavigate
from controls.goal_helpers import move_robot_centric
from .ActionStatus import ActionStatus

class BasicActionBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a template behaviour used to create others.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        py_trees.blackboard.Client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(
            self, 
            node,
            name = "ActionBehaviour",
            goal: AUVNavigate = None,
            ) -> None:
            """
            Initializes the node and blackboard client for this behaviour.

            Inputs: rclpy.node.Node             : node - the ROS2 node to use for subscribing to topics 
                    str                         : name - the name of the behaviour 
                    auv_msgs.action.AUVNavigate : goal - the goal to send to the action client

            Outputs: None
            """   
            super().__init__(name)
            self.node = node
            self.name = name
            self.blackboard = py_trees.blackboard.Client(name=self.name)
            self.goal = goal
            
            # Initialize fields needed to keep track of goal
            self.sent_goal = False
            self.action_status = ActionStatus.NOT_SENT # Either succeeded, pending, failed or not sent
            self.is_waiting_for_result = False
            
        def setup(self) -> None:
            """
            Description: Sets up keys on the blackboard that this behaviour will use.
            """
            # Get the navigation client from the blackboard to send goals
            self.blackboard.register_key(key="/navigation_client/client", access=py_trees.common.Access.READ)
            self.navigation_client = self.blackboard.navigation_client.client 
            self.navigation_client.client_wait_for_server(timeout_sec=5.0) # Ensure the action server is ready

            # Get the pose values from the blackboard
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
