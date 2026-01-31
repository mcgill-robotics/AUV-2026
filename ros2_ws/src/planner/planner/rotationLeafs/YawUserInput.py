import py_trees
from py_trees.common import Status, Access
from py_trees.blackboard import Client

class userInputYaw(py_trees.behaviour.Behaviour):
	""" 
	This behaviour gets user input for target yaw angle 
	and writes it to the blackboard for other behaviours to use
	"""
	def __init__(self, node, name="userInputYaw"):
		"""
		Initializes the userInputYaw behaviour. Initializes the blackboard
		with a goal target_yaw key, which will be used by other behaviours in the tree.

		Arguments:
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
		self.blackboard.register_key(key="/goal/target_yaw", access=py_trees.common.Access.WRITE)
		self.blackboard.register_key(key="/goal/target_yaw", access=py_trees.common.Access.READ)
		self.blackboard.goal.target_yaw = None # Initialize to None to allow for a tick to read a value on the first pass of this leaf

	def update(self):
		"""
		This method is called on every tick of the behaviour tree. It checks if the target_yaw on the blackboard
		is None (no input yet) or a string (target has been achieved). If there are no target yaw set, it prompts the user
		to input a target yaw angle in degrees, which it then writes to the blackboard.

		Arguments: None

		Outputs: py_trees.common.Status.SUCCESS once a target yaw is set
		"""
		# Check if target_yaw is a string, meaning the target has been reached. This allows to give a new 
		# target after one is done.
		if self.blackboard.goal.target_yaw is None or isinstance(self.blackboard.goal.target_yaw, str):
			self.node.get_logger().info("Waiting for user input for target Yaw...")
			target_yaw = float(input("Enter a target Yaw in degrees: "))
			self.blackboard.goal.target_yaw = target_yaw

		return Status.SUCCESS

		
