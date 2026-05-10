# Python dependencies
import py_trees
import math

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.node import Node

# AUV dependencies
from controls import navigation_client
from controls.goal_helpers import set_depth, set_global_yaw, move_robot_centric

# Planner dependencies
from ..mission_behaviour_components import BasicActionBehaviour

class TestMoveForwardBehaviour(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the test translation mission
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("TestMoveForwardBehaviour", memory=True)



        # Build the full mission sequence
        # 1. Move Forward
        forward_move_leaf = BasicActionBehaviour("Move to forward", move_robot_centric(forward=1.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))



        self.add_children([
            forward_move_leaf
            ])
