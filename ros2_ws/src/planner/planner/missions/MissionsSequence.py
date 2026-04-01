import py_trees
import py_trees_ros
import rclpy
from controls import navigation_client
from std_msgs.msg import Int32
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from controls.goal_helpers import set_depth, set_global_yaw, move_global
from ..SensorsBehaviour import SensorsBehaviour
from ..utils.BasicActionBehaviour import BasicActionBehaviour
from ..utils.TimerBehaviour import TimerBehaviour
from ..utils.MissionCompleteBehaviour import MissionCompleteBehaviour
from .preQual.OrbitQualification import OrbitQualificationMission
from .preQual.RectangleQualification import RectangleQualificationMission
from .preQual.TestMoveForwardBehaviour import TestMoveForwardBehaviour
from .preQual.TestYawBehaviour import TestYawBehaviour
from .preQual.TestDiveBehaviour import TestDiveBehaviour
from .preQual.TranslationRectangle import TranslationRectangleMission
import math

class MissionSequence(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the rectangle pre-qualification mission
    """
    def __init__(self, node):
        super().__init__("MissionSequence", memory=False)

        # Build the full mission sequence
        mission_choice = MissionChoiceBehaviour(node, name="Mission_Choice")
        all_missions = py_trees.composites.Selector("All missions", memory=True)

        pre_qual_rectangle = RectangleQualificationMission(node)
        pre_qual_orbit = OrbitQualificationMission(node)
        test_move_forward = TestMoveForwardBehaviour(node)
        test_dive = TestDiveBehaviour(node)
        test_yaw = TestYawBehaviour(node)
        translation_rectangle = TranslationRectangleMission(node)
        all_missions.add_children([pre_qual_orbit, 
            pre_qual_rectangle, 
            test_move_forward,
            test_dive, 
            test_yaw,
            translation_rectangle])

        self.add_children([mission_choice, 
            all_missions])

class MissionChoiceBehaviour(py_trees.behaviour.Behaviour):
    """ 
    This behaviour gets user input for mission choice
    and writes it to the blackboard for other mission behaviours to check.

    Fields: rclpy.Node: node         : the ROS2 node for logging and debugging purposes
    py_trees.blackboard blackboard   : the blackboard client
    """
    def __init__(self, node, name="MissionChoiceUser"):
        """
		Initializes the MissionChoiceUser behaviour. 

		Inputs:
			rclpy.Node: node -- The ROS2 node to use for logging and debugging purposes
			str: name        -- The name of the behaviour (default: "userInputYaw")
		"""
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.message_shown = False
	
    def setup(self):
        """
        Description: Sets up keys on the blackboard that this behaviour will use.
        """
        # Behaviour Tree bb setup in case of hardware setup or ros2 node setup
        self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.READ)
        self.blackboard.mission_choice = None
        
        # Create a ros2 subsriber to listen to user choices
        self.mission_choice_subscriber = self.node.create_subscription(Int32, 
                                                                       "/mission_selector", 
                                                                       self.mission_choice_callback, 
                                                                       10)
    
    def initialise(self):
        """Called every time this behavior transitions is not RUNNING."""
        self.message_shown = False
    
    def update(self):
        """
		This method is called on every tick of the behaviour tree. This behaviour gets the user's input
                to run a different missions.

		Inputs: None

		Outputs: py_trees.common.Status.SUCCESS once a valid input is chosen
		"""
        # Note that after a mission is completed, the respective mission will set this key back to None
        if self.blackboard.mission_choice is None:
            if not self.message_shown:
                self.node.get_logger().info("Waiting for mission choice. Publish once an Int32 to topic /mission_selector ...\n \
                            1: Orbit Prequal\n \
                            2: Rectangle Prequal\n \
                            3: Basic Move forward (1.0m relative to Dougie)\n \
                            4: Basic Dive (Down 1.5m)\n \
                            5: Basic Yaw (180 deg)\n \
                            6: Translation Rectangle (no yaw)")
                self.message_shown = True
            return py_trees.common.Status.RUNNING
           
        return py_trees.common.Status.SUCCESS
    
    def mission_choice_callback(self, msg: Int32):
        """
        This callback sets the mission_choice key on the blackboard to match 
        the topic's choice

        Input: 
            std_msgs.msg.Int32 : msg - The message sent on the topic /mission_choice representing the user's mission choice
        
        Outputs: None
        """
        if not (0 < msg.data < 7):
                self.node.get_logger().warn("Input must be an integer between 1 and 6 inclusively!")
                return
        
        self.blackboard.mission_choice = msg.data
        