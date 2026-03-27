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

class MissionChoiceBehaviour(py_trees.behaviour.Behaviour):
    """ 
	This behaviour gets user input for mission choice
	and writes it to the blackboard for other mission behaviours to check

	Fields: rclpy.Node: node         : the ROS2 node for logging and debugging purposes
	py_trees.blackboard blackboard   : the blackboard client for reading/writing the target yaw
	"""

	def __init__(self, node, name="userInputYaw"):
		"""
		Initializes the userInputYaw behaviour. Initializes the blackboard
		with a goal target_yaw key, which will be used by other behaviours in the tree.

		Inputs:
			rclpy.Node: node -- The ROS2 node to use for logging and debugging purposes
			str: name        -- The name of the behaviour (default: "userInputYaw")
		"""
		super().__init__(name)
		self.node = node
		self.blackboard = py_trees.blackboard.Client(name=self.name)
		
	def setup(self):
		"""
		Description: Sets up keys on the blackboard that this behaviour will use.
		"""
		# Behaviour Tree bb setup in case of hardware setup or ros2 node setup
		self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.WRITE)
		self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.READ)
		self.blackboard.mission_choice = None

	def update(self):
		"""
		This method is called on every tick of the behaviour tree. This behaviour gets the user's input
                to run a different missions.

		Inputs: None

		Outputs: py_trees.common.Status.SUCCESS once a valid input is chosen
		"""
		# Check if target_yaw is a string, meaning the target has been reached. This allows to give a new 
		# target after one is done.
		if self.blackboard.mission_choice is None:
			self.node.get_logger().info("Waiting for mission choice...\n \
                        1: Orbit Prequal\n \
                        2: Rectangle Prequal\n \
                        3: Basic Move forward\n \
                        4: Basic Dive")
			user_input = (input("Enter a target Yaw in degrees: "))
			self.blackboard.mission_choice = user_input

		return py_trees.common.Status.SUCCESS