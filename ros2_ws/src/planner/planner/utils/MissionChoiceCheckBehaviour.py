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

class MissionChoiceCheckBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a template behaviour used to create others.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        py_trees.blackboard.Client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(self, choice: int, name="MissionChoiceCheckBehaviour") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: rclpy.node.Node    : node - the ROS2 node to use for subscribing to topics 
                        str                : name - the name of the behaviour 

                Outputs: None
                """   
                super().__init__(name)
                self.blackboard = py_trees.blackboard.Client(name=self.name)
                self.choice = choice

        def setup(self) -> None:
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