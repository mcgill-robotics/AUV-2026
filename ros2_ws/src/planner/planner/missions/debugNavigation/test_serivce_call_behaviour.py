# Python dependencies
import math
import py_trees

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.node import Node

# AUV dependencies

# Planner dependencies
from ..mission_behaviour_components import MissionChoiceCheckBehaviour, \
       MissionCompleteBehaviour, BasicTriggerServiceBehaviour

class TestServiceCallBehaviour(py_trees.composites.Sequence):
    """
    Attempt a service call to reset the dead reckoning.
    """
    def __init__(self, node):
        super().__init__("Reset Dead Reckoning", memory=True)
        self.node = node

        # 0 Check if user chose this mission
        mission_choice_check = MissionChoiceCheckBehaviour(name="Reset Dead Reckoning", choice=7)

        # 1 Reset dead reckoning 
        reset_dr = BasicTriggerServiceBehaviour(node=node, name="Reset Dead Reckoning", service_name="reset_dead_reckoning")

        # 2. Reset mission choice
        mission_choice_reset = MissionCompleteBehaviour(node, "Completed Reset Dead Reckoning")

        self.add_children([mission_choice_check,
            reset_dr,
            mission_choice_reset
            ])
