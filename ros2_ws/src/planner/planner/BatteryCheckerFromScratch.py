import py_trees
from py_trees.common import Status, Access
from py_trees.blackboard import Client
#need to import std_msgs properly

class BatteryChecker(py_trees.behaviour.Behaviour):
	
	def __init__(self, name="BatteryChecker", threshold=14.5):
		super().__init__(name)
		self.threshold = threshold
		self.msg = None

	def setup(self, node: rclcpy.node.Node):
		"""
		Parameters: rclcpy.node.Node

		Description: Sets up a subscriber to listen to battery topics 
		and put those values on the BT blackboard for other Behaviours (tree leafs)
		to use

		Outputs: Nothing
		"""
		
		# Behaviour Tree bb setup
		self.blackboard_client_battery = py_trees.blackboard.Client(name=self.name)

		self.blackboard_client_battery.register_key("battery_level1", Access.WRITE)
		self.blackboard_client_battery.register_key("battery_level2", Access.WRITE)

		# Make ROS2 node setups
		self.sub_batteries_voltage = node.create_subscription(std_msgs/msg/Float32MultiArray,
			/power/batteries/voltage, 
			self.callback,
			10
		)

	def callback(self, msg: std_msgs/msg/Float32MultiArray):
		"""
		Parameters: Batteries voltage message

		Description: Sets the class msg to be processed in the update of the class,
		logic not handled here because we want the tree to evolve with ticks on the tree
		and not with a subscribers innate callback

		Outputs: NATTA
		"""

		self.msg = msg
	
	# Battery logic to handle tmr, I am big tired
