import py_trees
import rclpy
from py_trees.blackboard import Blackboard
from py_trees.common import Status, Access
from auv_msgs.msg import ThrusterForces

class YawMovement(py_trees.behaviour.Behaviour):
	"""
	This behaviour reads the yaw error from the blackboard and publishes
	the appropriate thruster commands to rotate the robot towards the target yaw.
	"""
	def __init__(self, node, name="YawMovement"):
		super().__init__(name)
		self.node = node
		self.Kp = 1.0

		self.blackboard_odom = py_trees.blackboard.Client(name=self.name)
		

	def setup(self):
		"""
		Description: Sets up a subscriber to the current orientation
		of the IMU and a publisher to propulsion to move the robot to
		a given YAW

		Parameters: None

		Outputs: Nothing
		"""
		self.blackboard_odom.register_key(key="/odom/imu/yaw", access=Access.READ)
		self.blackboard_odom.register_key(key="/goal/yaw_error", access=Access.READ)
		self.blackboard_odom.register_key(key="/goal/target_yaw", access=Access.READ)
		
		# For the sim, look for topics in _Project/Scenes/25x50Pool.unity
		self.pub = self.node.create_publisher(ThrusterForces, "/propulsion/forces", 10)
		

	def update(self):
		"""
		Description: On every tick, this method reads the yaw error from the blackboard,
		calculates the required thruster forces using a proportional controller, and publishes
		the thruster commands to rotate the robot. Note that the thruster Unity effort magnitudes
		are yet to be understood by me :( . Hence arbitrary Kp value for now.

		Parameters: None

		Outputs: A Status of RUNNING indicating that the rotation is in progress. A Status of SUCCESS
		is returned when the target yaw is reached and thrusters are stopped.
		"""
		# Check if target yaw is marked as "Done"
		if self.blackboard_odom.goal.target_yaw == "Done":
			# Stop thrusters when done
			msg = ThrusterForces()
			msg.front_left = 0.0
			msg.back_left = 0.0
			msg.front_right = 0.0
			msg.back_right = 0.0

			# Zero vertical thrusters
			msg.heave_front_left = 0.0
			msg.heave_back_left = 0.0
			msg.heave_front_right = 0.0
			msg.heave_back_right = 0.0

			self.pub.publish(msg)

			self.node.get_logger().info("YawMovement: Target reached, stopping thrusters.")
			return py_trees.common.Status.SUCCESS

		effort = self.blackboard_odom.goal.yaw_error * self.Kp
		self.node.get_logger().info(f"Applying yaw effort: {effort}")
		msg = ThrusterForces()
		msg.front_left = +effort
		msg.back_left = -effort
		msg.front_right = -effort
		msg.back_right = +effort

		# Zero vertical thrusters
		msg.heave_front_left = 0.0
		msg.heave_back_left = 0.0
		msg.heave_front_right = 0.0
		msg.heave_back_right = 0.0

		self.pub.publish(msg)


		return py_trees.common.Status.RUNNING

		'''
		try:
			test = self.blackboard_odom.goal.yaw_error
		except:
			self.node.get_logger().info("YawMovement: No yaw error on blackboard yet, waiting...")
			return py_trees.common.Status.RUNNING
		'''
		

