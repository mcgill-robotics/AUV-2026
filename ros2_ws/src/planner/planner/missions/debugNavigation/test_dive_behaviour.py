# Python dependencies
import py_trees
import math

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.node import Node

# AUV dependencies
from controls.goal_helpers import set_depth

# Planner dependencies
from ..mission_behaviour_components import BasicActionBehaviour, MissionChoiceCheckBehaviour, \
       MissionCompleteBehaviour


class TestDiveBehaviour(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the test translation mission
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("TestDiveBehaviour", memory=True)

        # 0 Check if user input the desired mission choce
        """
        1: Orbit Prequal
        2: Rectangle Prequal
        3: Basic Move forward
        4: Basic Dive
        5: Basic Yaw
        """
        mission_choice_check = MissionChoiceCheckBehaviour(name="Test Dive", choice=4)

        # Build the full mission sequence
        # 1. Dive to -1.5m
        dive_leaf = BasicActionBehaviour("Test Dive", set_depth(z=-1.5, tolerance=position_tolerance, \
                                                                      hold_time=hold_time, timeout=timeout))
        
        # 2. Reset the user mission choice to allow for new mission to be selected
        mission_choice_reset = MissionCompleteBehaviour("Completed Test Dive")

        self.add_children([mission_choice_check,
            dive_leaf, 
            mission_choice_reset
            ])
