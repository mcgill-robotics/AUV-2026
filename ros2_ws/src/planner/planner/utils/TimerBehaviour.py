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
import time

class TimerBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a timer to be implemented. The timer is dictated by measuring a time.time()
        interval and checking if it is above a desired threshold upon a new tick's arrival. This not fully accurate  
        method is acceptable since the current scope of this TimerBehaviour is to allow time to untether Douglas

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        py_trees.blackboard.Client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(self, node, timer: float, name="sensorsLeaf") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: rclpy.node.Node    : node - the ROS2 node to use for subscribing to topics 
                        str                : name - the name of the behaviour 
                        float              : timer - the desired duration of the timer

                Outputs: None
                """   
                super().__init__(name)
                self.timer = timer
                self.start_time = 0.0
                self.timer_started = False
                self.node = node
        
        def initialise(self) -> None:
                """
                Whenever the Behaviour is not in RUNNING state, reset the timer to not started

                Inputs: None

                Outputs: None
                """
                self.timer_started = False

        def update(self) -> py_trees.common.Status:
                """
                Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

                Ibputs: None

                Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                         py_trees.common.Status.FAILURE if it failed, or 
                         py_trees.common.Status.RUNNING if it is still running.
                 
                """
                if not self.timer_started:
                        # Set the initial time at start of time counting
                        self.start_time = time.time()
                        self.timer_started = True
                        return py_trees.common.Status.RUNNING
                
                # Upon each tick, check if interval of time has been passed
                self.node.get_logger().info(f"Time passed: {(time.time() - self.start_time)}")
                if (time.time() - self.start_time) > float(self.timer):
                        return py_trees.common.Status.SUCCESS
                
                return py_trees.common.Status.RUNNING