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
from ..mission_behaviour_components import BasicActionBehaviour


class TestDiveBehaviour(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the test translation mission
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("TestDiveBehaviour", memory=True)



        # Build the full mission sequence
        # 1. Dive to -1.5m
        dive_leaf = BasicActionBehaviour("Test Dive", set_depth(z=-1.5, tolerance=position_tolerance, \
                                                                      hold_time=hold_time, timeout=timeout))
        


        self.add_children([
            dive_leaf
            ])
