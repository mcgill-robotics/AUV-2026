# Python dependencies
import math
import py_trees

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.node import Node

# AUV dependencies
from controls.goal_helpers import rotate_relative

# Planner dependencies
from ..mission_behaviour_components import BasicActionBehaviour, MissionChoiceCheckBehaviour, \
       MissionCompleteBehaviour


class TestYawBehaviour(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the test translation mission
    """
    def __init__(self, yaw_tolerance: float, hold_time: float, timeout: float):
        super().__init__("TestYawBehaviour", memory=True)

        # 0 Check if user input the desired mission choce
        """
        1: Orbit Prequal
        2: Rectangle Prequal
        3: Basic Move forward
        4: Basic Dive
        5: Basic Yaw
        """
        mission_choice_check = MissionChoiceCheckBehaviour(name="Check Yaw Test", choice=5)

        # Build the full mission sequence
        # 1. Rotate 180 deg Yaw
        yaw_rotate_leaf = BasicActionBehaviour("Yaw Test", rotate_relative(dyaw_rad=math.pi, tolerance=yaw_tolerance, hold_time=hold_time, timeout=timeout))

        # 2. Reset the user mission choice to allow for new mission to be selected
        mission_choice_reset = MissionCompleteBehaviour("Completed Yaw Test")

        self.add_children([mission_choice_check,
            yaw_rotate_leaf, 
            mission_choice_reset
            ])
