# Python dependencies
import math
import py_trees

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.node import Node

# Planner dependencies
from ..mission_behaviour_components import BasicTriggerServiceBehaviour

class TestServiceCallBehaviour(py_trees.composites.Sequence):
    """
    Attempt a service call to reset the dead reckoning.
    """
    def __init__(self):
        super().__init__("Reset Dead Reckoning", memory=True)



        # 1 Reset dead reckoning 
        reset_dr = BasicTriggerServiceBehaviour(name="Reset Dead Reckoning", service_name="reset_dead_reckoning")



        self.add_children([
            reset_dr
            ])
