import py_trees
import py_trees_ros
import rclpy
from controls import navigation_client
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from controls.goal_helpers import set_depth, set_global_yaw, move_robot_centric
from ...SensorsBehaviour import SensorsBehaviour
from ...utils.BasicActionBehaviour import BasicActionBehaviour
from ...utils.MissionChoiceCheckBehaviour import MissionChoiceCheckBehaviour
from ...utils.MissionCompleteBehaviour import MissionCompleteBehaviour
from ...utils.TimerBehaviour import TimerBehaviour
import math

class TestMoveForwardBehaviour(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the test translation mission
    """
    def __init__(self, node):
        super().__init__("TestMoveForwardBehaviour", memory=True)

        # 0 Check if user input the desired mission choce
        """
        1: Orbit Prequal
        2: Rectangle Prequal
        3: Basic Move forward
        4: Basic Dive
        5: Basic Yaw
        """
        mission_choice_check = MissionChoiceCheckBehaviour(name="Test Move Forward", choice=3)

        # Build the full mission sequence
        # 1. Move Forward
        forward_move_leaf = BasicActionBehaviour(node, "Move to forward", move_robot_centric(forward=1.0, tolerance=0.5, hold_time=1.0))

        # 2. Reset the user mission choice to allow for new mission to be selected
        mission_choice_reset = MissionCompleteBehaviour(node, "Completed Test Move Forward")

        self.add_children([mission_choice_check, 
            forward_move_leaf, 
            mission_choice_reset
            ])
