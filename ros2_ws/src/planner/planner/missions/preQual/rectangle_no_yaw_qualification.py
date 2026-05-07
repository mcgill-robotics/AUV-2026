# Python dependencies
import math

# ROS dependencies
import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node

# AUV dependencies
from controls import navigation_client
from controls.goal_helpers import set_depth, move_global

# Planner dependencies
from ..mission_behaviour_components import BasicActionBehaviour, MissionChoiceCheckBehaviour, \
       MissionCompleteBehaviour, TimerBehaviour


class TranslationRectangleMission(py_trees.composites.Sequence):
    """
    Same rectangle path as the RectangleQualification mission but with
    NO yaw commands - the AUV only translates between waypoints.
    Useful for pool-testing the planar controller without attitude changes.
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("TranslationRectangle", memory=True)

        # 0 Check if user chose this mission
        mission_choice_check = MissionChoiceCheckBehaviour(name="TranslationRectangleCheck", choice=6)

        # 1 Wait for 10 seconds before starting the mission
        timer = TimerBehaviour(timer_duration=10.0, name="Translation Rectangle Timer")

        # 2. Dive to -1.5m
        dive_leaf = BasicActionBehaviour("Dive", set_depth(z=-1.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 3. Move past the gate (5m forward)
        past_gate = BasicActionBehaviour("Move past gate", move_global(x=5.0, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 4. Ascend to -0.5m (clear the gate)
        ascend_past_gate = BasicActionBehaviour("Ascend past gate", set_depth(z=-0.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 5. Go to rectangle start point
        go_start = BasicActionBehaviour("Move to rectangle start", move_global(x=11.5, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 4. Bottom left corner
        bottom_left = BasicActionBehaviour("Bottom Left", move_global(x=11.5, y=-1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 5. Top left corner
        top_left = BasicActionBehaviour("Top Left", move_global(x=14.5, y=-1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 6. Top right corner
        top_right = BasicActionBehaviour("Top Right", move_global(x=14.5, y=1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 7. Bottom right corner
        bottom_right = BasicActionBehaviour("Bottom Right", move_global(x=11.5, y=1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 8. Back to start
        go_start_2 = BasicActionBehaviour("Back to rectangle start", move_global(x=11.5, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # Move back to gate area (5m)
        back_to_gate = BasicActionBehaviour("Back to gate", move_global(x=5.0, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # Descend to -1.5m to go back under the gate
        descend_at_gate = BasicActionBehaviour("Descend at gate", set_depth(z=-1.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # Return to origin
        return_leaf = BasicActionBehaviour("Return to Origin", move_global(x=0.0, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 10. Ascend to surface
        ascend_leaf = BasicActionBehaviour("Ascend to Surface", set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 11. Reset mission choice
        mission_choice_reset = MissionCompleteBehaviour("Completed Translation Rectangle")

        self.add_children([mission_choice_check,
            timer,
            dive_leaf,
            past_gate,
            ascend_past_gate,
            go_start, 
            bottom_left,
            top_left, 
            top_right, 
            bottom_right,
            go_start_2,
            back_to_gate,
            descend_at_gate,
            return_leaf,
            ascend_leaf,
            mission_choice_reset
            ])
