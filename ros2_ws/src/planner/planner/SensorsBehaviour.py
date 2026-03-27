import py_trees
import rclpy
import py_trees_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from py_trees.common import Status, Access
from py_trees.blackboard import Client
import auv_msgs.msg
import geometry_msgs.msg

class SensorsBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour reads sensor data from the AUV's pose, twist and object_map topics 
        and writes them to the blackboard for other behaviours to use.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        py_trees.blackboard.Client: blackboard         : the blackboard client for reading/writing sensors data
        geometry_msgs.msg.Pose: current_pose          : the latest pose message received from the topic
        geometry_msgs.msg.Twistcurrent_twist          : the latest twist message received from the topic
        auv_msgs.msg.ObjectMapcurrent_object_map      : the latest object map message received from the topic
        """

        def __init__(self, node, name="sensorsLeaf") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: rclpy.node.Node    : node - the ROS2 node to use for subscribing to topics 
                        str                : name - the name of the behaviour 

                Outputs: None
                """   
                super().__init__(name)
                self.node = node
                self.blackboard = py_trees.blackboard.Client(name=self.name)
                self.current_pose = None
                self.current_twist = None
                self.current_object_map = None


        def setup(self, **kwargs) -> None:
                """
                Description: Sets up keys on the blackboard that this behaviour will use.
                """
                # Behaviour Tree bb setup in case of hardware setup or ros2 node setup
                self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.WRITE)
                self.blackboard.register_key(key="/sensors/twist", access=py_trees.common.Access.WRITE)
                self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.WRITE)
                
                self.blackboard.sensors.pose = None
                self.blackboard.sensors.twist = None
                self.blackboard.vision.object_map = None

                # Choose Best Effort reliability for sensor data since having latest data is priority
                qos = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT
                )
                
                # If in sim and desire to use absolute pose and twist, subscribe to ground truth topics
                self.node.declare_parameter("sim", False)
                self.node.declare_parameter("use_ground_truth", False)

                use_sim = self.node.get_parameter("sim").get_parameter_value().bool_value
                use_ground_truth = self.node.get_parameter("use_ground_truth").get_parameter_value().bool_value

                topic_pose = "state/pose"
                topic_twist = "auv_frame/dvl/velocity"

                # Currently can't think of a better way to not use two params 
                #(since use_ground_truth is only valid if in sim, but in sim you might still wanna use state estimation)
                if use_sim and use_ground_truth: 
                        topic_pose = "/auv/ground_truth/pose"
                        topic_twist = "/auv/ground_truth/twist"
                        
                self.sub_pose = self.node.create_subscription(geometry_msgs.msg.PoseStamped,
                        topic_pose, 
                        self.pose_callback,
                        qos_profile=qos)

                self.sub_twist = self.node.create_subscription(geometry_msgs.msg.TwistStamped,
                        topic_twist, 
                        self.twist_callback,
                        qos_profile=qos)
                
                self.sub_vision = self.node.create_subscription(auv_msgs.msg.VisionObjectArray,
                        "/vision/object_map", 
                        self.object_map_callback,
                        qos_profile=qos)

        def update(self) -> py_trees.common.Status:
                """
                Description: This function is called every tick. It writes the latest sensor data to the blackboard.
                It is important to have the latest sensor data on the blackboard from each tick instance instead of 
                having other behaviours subscribe to the topics directly, because it ensures that all behaviours are working with the same data each tick.

                Ibputs: None

                Outputs: py_trees.common.Status.SUCCESS
                """
                self.blackboard.sensors.pose = self.current_pose
                self.blackboard.sensors.twist = self.current_twist
                self.blackboard.vision.object_map = self.current_object_map
                
                # Current Debugger output, to look into PyTrees's own blackboard visualizer
                #self.node.get_logger().info(f"Pose: {self.blackboard.sensors.pose}")
                #self.node.get_logger().info(f"Twist: {self.blackboard.sensors.twist}")
                #self.node.get_logger().info(f"Object Map: {self.blackboard.vision.object_map}")
                #self.node.get_logger().info("--------------------------------------------------")      

                return py_trees.common.Status.RUNNING

        def pose_callback(self, msg: geometry_msgs.msg.PoseStamped) -> None:
                """
                Description: Callback function for the pose subscriber. It updates the field of the class current_pose
                with the latest message from the topic.

                Inputs: msg: geometry_msgs.msg.PoseStamped - the latest message from the pose topic
                Outputs: None
                """
                self.current_pose = msg
        
        def twist_callback(self, msg: geometry_msgs.msg.TwistStamped) -> None:
                """
                Description: Callback function for the twist subscriber. It updates the field of the class current_twist
                with the latest message from the topic.

                Inputs: msg: geometry_msgs.msg.TwistStamped - the latest message from the twist topic
                Outputs: None
                """
                print(msg)
                self.current_twist = msg
        
        def object_map_callback(self, msg: auv_msgs.msg.VisionObjectArray) -> None:
                """
                Description: Callback function for the object map subscriber. It updates the field of the class current_object_map
                with the latest message from the topic.

                Inputs: msg: auv_msgs.msg.VisionObjectArray - the latest message from the object map topic
                Outputs: None
                """
                self.current_object_map = msg
