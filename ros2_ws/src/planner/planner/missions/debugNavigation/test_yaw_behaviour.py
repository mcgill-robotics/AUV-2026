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
from ..mission_behaviour_components import BasicActionBehaviour


class TestYawBehaviour(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the test translation mission
    """
    def __init__(self, angular_tolerance: float, hold_time: float, timeout: float):
        super().__init__("TestYawBehaviour", memory=True)



        # Build the full mission sequence
        # 1. Rotate 180 deg Yaw
        yaw_rotate_leaf = BasicActionBehaviour("Yaw Test", rotate_relative(dyaw_rad=math.pi, tolerance=angular_tolerance, hold_time=hold_time, timeout=timeout))



        self.add_children([
            yaw_rotate_leaf
            ])
