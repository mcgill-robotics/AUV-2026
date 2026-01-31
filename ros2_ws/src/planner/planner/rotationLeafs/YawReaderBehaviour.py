import rclpy
from rclpy.node import Node
import py_trees
from rclpy.qos import QoSProfile, ReliabilityPolicy
from py_trees.common import Status, Access
from py_trees.blackboard import Blackboard
import sensor_msgs.msg 
import geometry_msgs.msg
import math

class YawReader(py_trees.behaviour.Behaviour):
	""" 
	This behaviour reads the current quaternion from the IMU,
	converts it to YAW and compares it to the user defined target YAW.
	It then writes the YAW error to the blackboard for other behaviours to use.
	"""
	
	def __init__(self, node: Node, name="YawReader"):
		# To change to ODOM when state aggregator is done, for now use IMU
		super().__init__(name)
		self.node = node
		self.node.get_logger().info("YawReader Behaviour Initialized")
		self.latest_imu_msg = None
		self.name = name
		self.tolerance = 0.05
		self.prev_yaw = None
		self.yaw_unwrapped = None

		self.blackboard_odom = py_trees.blackboard.Client(name=self.name)
		

	def setup(self):
		"""
		Description: Sets up a subscriber to listen to IMU message. Sets up
		the needed blackboard keys

		Arguments: None

		Outputs: Nothing
		"""
		self.node.get_logger().info("YawReader Behaviour Initialized2")

		# Make ROS2 node setups
		self.blackboard_odom.register_key(key="/odom/imu/yaw", access=Access.WRITE)
		self.blackboard_odom.register_key(key="/odom/imu/yaw", access=Access.READ)
		self.blackboard_odom.register_key(key="/goal/target_yaw", access=Access.READ)
		self.blackboard_odom.register_key(key="/goal/target_yaw", access=Access.WRITE)
		self.blackboard_odom.register_key(key="/goal/yaw_error", access=Access.WRITE)
		self.blackboard_odom.odom.imu.yaw = None

		qos = QoSProfile(
			depth=10,
			reliability=ReliabilityPolicy.BEST_EFFORT
		)
		self.sub_imu = self.node.create_subscription(geometry_msgs.msg.QuaternionStamped,
			"/auv/ground_truth/orientation", 
			self.yaw_from_quaternion,
			qos_profile=qos)
		

	def yaw_from_quaternion(self, msg):
		"""
		Description: Callback function for IMU subscriber. Extracts yaw from quaternion
		and writes it to the blackboard. Sets the latest_imu_msg for processing in update. It
		serves as a flag that we have received at least one IMU message.

		Arguments: 
			geometry_msgs.msg.QuaternionStamped: msg (refer to ROS docs) -- The msg from the IMU

		Outputs: Nothing
		"""
		self.latest_imu_msg = msg

		# Use yaw extraction from quaternion
		siny_cosp = 2.0 * (msg.quaternion.w * msg.quaternion.z + msg.quaternion.x * msg.quaternion.y)
		cosy_cosp = 1.0 - 2.0 * (msg.quaternion.y * msg.quaternion.y + msg.quaternion.z * msg.quaternion.z)
		yaw = math.atan2(siny_cosp, cosy_cosp) 
		
		if self.prev_yaw is None:
			self.yaw_unwrapped = yaw
		else:
			delta = yaw - self.prev_yaw
			delta = math.atan2(math.sin(delta), math.cos(delta))
			self.yaw_unwrapped += delta

		self.prev_yaw = yaw
		self.blackboard_odom.odom.imu.yaw = yaw
	
	def update(self):
		"""
		Description: On every tick, this method checks the current yaw against the target yaw.
		It computes the yaw error and writes it to the blackboard for other behaviours to use.

		Arguments: None

		Outputs: A Status of either SUCCESS, RUNNING, FAILURE depending on whether the target yaw
		has been reached within tolerance.
		"""
		if self.blackboard_odom.goal.target_yaw == "Done":
			return py_trees.common.Status.SUCCESS

		if self.latest_imu_msg is None:
			return py_trees.common.Status.RUNNING

		self.node.get_logger().info(f"Target Yaw: {float(self.blackboard_odom.goal.target_yaw) * (math.pi / 180.0)}, Current Yaw: {self.yaw_unwrapped}")
		error = float(self.blackboard_odom.goal.target_yaw) * (math.pi / 180.0) - self.yaw_unwrapped # For some reason need to recast as float, need to investigate later

		if abs(error) < self.tolerance:
			# Set target_yaw to "Done" to signal completion and allow user to input a new target in YawUserInput Behaviour
			self.blackboard_odom.goal.target_yaw = "Done"
			return py_trees.common.Status.SUCCESS

		self.blackboard_odom.goal.yaw_error = error
		return py_trees.common.Status.RUNNING



