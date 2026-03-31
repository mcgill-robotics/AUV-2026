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

class MissionCompleteBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour serves as resetting the user input when a mission is finished.
        When the user input is empty, the user will once again get the choice to choose a mission

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        py_trees.blackboard.Client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(self, node, name="MissionCompleteBehaviour") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: rclpy.node.Node    : node - the ROS2 node to use for subscribing to topics 
                        str                : name - the name of the behaviour 

                Outputs: None
                """   
                super().__init__(name)
                self.node = node
                self.blackboard = py_trees.blackboard.Client(name=self.name)

        def setup(self) -> None:
                """
                Description: Sets up keys on the blackboard that this behaviour will use.
                """
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