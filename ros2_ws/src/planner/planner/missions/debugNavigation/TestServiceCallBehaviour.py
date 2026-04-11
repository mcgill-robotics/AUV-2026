import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node
from controls.goal_helpers import set_depth, set_global_yaw, move_robot_centric
from ...utils.BasicActionBehaviour import BasicActionBehaviour
from ...utils.MissionChoiceCheckBehaviour import MissionChoiceCheckBehaviour
from ...utils.MissionCompleteBehaviour import MissionCompleteBehaviour
from ...utils.TimerBehaviour import TimerBehaviour

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
