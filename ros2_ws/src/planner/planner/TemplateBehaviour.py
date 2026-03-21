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

class TemplateBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a template behaviour used to create others.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        py_trees.blackboard.Client: blackboard        : the blackboard client for reading/writing sensors data
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
                self.action_client = ActionClient(self.node, AUVNavigate, '/motion/navigate')
                self.sent_goal = False


        def setup(self) -> None:
                """
                Description: Sets up keys on the blackboard that this behaviour will use.
                """
                # EXAMPLE FROM SensorsBehaviour.py :Behaviour Tree bb setup in case of hardware setup or ros2 node setup
                #self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.WRITE)
                #self.blackboard.register_key(key="/sensors/twist", access=py_trees.common.Access.WRITE)
                #self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.WRITE)
                
                self.action_client.wait_for_server(timeout_sec=5.0)
                
        def update(self) -> py_trees.common.Status:
                """
                Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

                Ibputs: None

                Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                         py_trees.common.Status.FAILURE if it failed, or 
                         py_trees.common.Status.RUNNING if it is still running.
                 
                """
                
                # EXAMPLE: Create a navigation goal and send it using the local action client
                self.node.get_logger().info("Template Behaviour Tick")
                if self.sent_goal == False:
                        goal_msg = move_robot_centric(forward=15.0, tolerance=0.5, yaw_tolerance=0.1, hold_time=3.0, timeout=30.0)
                        self.action_client.send_goal_async(goal_msg)
                        self.sent_goal = True
                        
                        self.sent_goal = True
                
                return py_trees.common.Status.RUNNING