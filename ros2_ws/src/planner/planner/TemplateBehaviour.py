import geometry_msgs
import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from py_trees.common import Status, Access
from py_trees.blackboard import Client
import auv_msgs.msg

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
                self.sent_goal = False


        def setup(self) -> None:
                """
                Description: Sets up keys on the blackboard that this behaviour will use.
                """
                # EXAMPLE FROM SensorsBehaviour.py :Behaviour Tree bb setup in case of hardware setup or ros2 node setup
                #self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.WRITE)
                #self.blackboard.register_key(key="/sensors/twist", access=py_trees.common.Access.WRITE)
                #self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.WRITE)
                
                self.blackboard.register_key(key="/navigation_client", access=py_trees.common.Access.READ)
                
        def update(self) -> py_trees.common.Status:
                """
                Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

                Ibputs: None

                Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                         py_trees.common.Status.FAILURE if it failed, or 
                         py_trees.common.Status.RUNNING if it is still running.
                 
                """
                
                # EXAMPLE: Create a navigation goal and send it using the navigation client from the blackboard
                self.node.get_logger().info("Template Behaviour Tick")
                if self.sent_goal == False:
                        nav_client = self.blackboard.navigation_client
                        goal_msg = auv_msgs.action.AUVNavigate.Goal()
                        goal_msg.target_pose.position.x = 15.0
                        goal_msg.target_pose.position.y = 0.0
                        goal_msg.target_pose.position.z = 0.0
                        goal_msg.target_pose.orientation.x = 0.0
                        goal_msg.target_pose.orientation.y = 0.0
                        goal_msg.target_pose.orientation.z = 0.0
                        goal_msg.target_pose.orientation.w = 1.0
                        
                        goal_msg.do_x = True
                        goal_msg.do_y = False
                        goal_msg.do_z = False
                        goal_msg.do_yaw = False
                        goal_msg.is_relative = True
                        goal_msg.is_local_frame = True
                        goal_msg.position_tolerance = 0.5
                        goal_msg.yaw_tolerance = 0.1
                        goal_msg.hold_time = 3.0
                        goal_msg.timeout = 30.0
                        nav_client.send_navigation_goal(goal_msg)
                        
                        self.sent_goal = True
                
                return py_trees.common.Status.RUNNING