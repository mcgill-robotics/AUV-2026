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

		self.blackboard_client_battery.register_key("/embedded/battery_level1", Access.WRITE)
		self.blackboard_client_battery.register_key("/embedded/battery_level2", Access.WRITE)

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

	def update(self) -> Status:
		""" 
		Parameters: None

		Description: On every tick after initialization, the Behaviour will write the
		most recent state of each battery's voltage on the blackboard. This can be helpful if 
		we want to turn certain tasks off depending on battery voltages. Anything voltages below
		a certain threshold (determined by Elec, currently around 15.0) will cause a shutdown
		of the tree

		Outputs: A Status of either SUCCESS, RUNNING, FAILURE 
		"""

		# Check if any message at all, in case the Tree is initialized before ROS2 node
		if (self.msg == None) 
			return Status.RUNNING

		# Extract the msg data and write it to the blackboard for other behaviours to use
		self.blackboard_client_battery.embedded.battery_level1 = self.msg.data[0]
		self.blackboard_client_battery.embedded.battery_level1 = self.msg.data[1]

		# Check that both batteries are under the threshold (might make it a param (prob should)) 
		# We check both because of elecs cool dual battery channel setup where one fall backs on the other

		if (self.blackboard_client_battery.embedded.battery_level1 < 15.0 && \
			self.blackboard_client_battery.embedded.battery_level2 < 15.0)
			return Status.FAILURE # Status.FAILURE will crash the root, which we plan to be the whole tree or start a recovery action from Dougie to surface
		
		# If any publishers in the Behaviour, publish here

		return Status.SUCCESS
			 
