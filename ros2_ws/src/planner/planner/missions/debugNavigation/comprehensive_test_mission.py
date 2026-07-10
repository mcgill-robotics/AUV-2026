# Python dependencies
import py_trees
import math

# ROS dependencies
from rclpy.node import Node

# AUV dependencies
from controls.goal_helpers import set_depth, set_global_yaw, move_global, set_attitude

# Planner dependencies
from ..mission_behaviour_components import BasicActionBehaviour, TimerBehaviour

class ComprehensiveTestMission(py_trees.composites.Sequence):
    """
    This PyTrees Sequence tests all degrees of freedom using absolute global
    coordinates to prevent odometry drift, explicitly confining movements to a single lane.
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("ComprehensiveTestMission", memory=True)

        test_depth = -0.25
        surface_depth = 0.0

        fwd_dis = 2.0
        timer_delay = 1.0
        # =====================================================================
        # 0. Initial Dive
        # =====================================================================
        dive = BasicActionBehaviour("Initial Dive to -1.0", set_depth(z=test_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        delay_0 = TimerBehaviour(timer_duration=timer_delay, name="Delay 0")

        # =====================================================================
        # 1-2. Surge Test (Forward and Backward)
        # =====================================================================
        surge_fwd = BasicActionBehaviour("Surge Forward", move_global(x=fwd_dis, y=0.0, z=test_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        delay_1 = TimerBehaviour(timer_duration=timer_delay, name="Delay 1")
        
        surge_bwd = BasicActionBehaviour("Surge Backward", move_global(x=0.0, y=0.0, z=test_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        delay_2 = TimerBehaviour(timer_duration=timer_delay, name="Delay 2")

        # =====================================================================
        # 3-5. Sway Test (Right and Left while rotated 90 degrees)
        # =====================================================================
        turn_90 = BasicActionBehaviour("Turn 90 Left", set_global_yaw(yaw_rad=math.pi/2, hold_time=hold_time, timeout=timeout))
        delay_3 = TimerBehaviour(timer_duration=timer_delay, name="Delay 3")
        
        # Moving to X=2.0 while facing Y forces a right sway
        sway_right = BasicActionBehaviour("Sway Right", move_global(x=fwd_dis, y=0.0, z=test_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        delay_4 = TimerBehaviour(timer_duration=timer_delay, name="Delay 4")
        
        # Moving back to X=0.0 while facing Y forces a left sway
        sway_left = BasicActionBehaviour("Sway Left", move_global(x=0.0, y=0.0, z=test_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        delay_5 = TimerBehaviour(timer_duration=timer_delay, name="Delay 5")

        turn_fwd_1 = BasicActionBehaviour("Face Forward", set_global_yaw(yaw_rad=0.0, hold_time=hold_time, timeout=timeout))
        delay_6 = TimerBehaviour(timer_duration=timer_delay, name="Delay 6")

        # =====================================================================
        # 6-7. Heave Test (Surface and Re-dive)
        # =====================================================================
        # heave_up = BasicActionBehaviour("Ascend to -0.2", set_depth(z=surface_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        # delay_7 = TimerBehaviour(timer_duration=2.0, name="Delay 7")
        
        # heave_down = BasicActionBehaviour("Dive to -1.0", set_depth(z=test_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        # delay_8 = TimerBehaviour(timer_duration=2.0, name="Delay 8")

        # =====================================================================
        # 8-10. Diagonal Test (Surge + Sway at 45 degrees)
        # =====================================================================
        turn_45 = BasicActionBehaviour("Turn 45 Left", set_global_yaw(yaw_rad=math.pi/4, hold_time=hold_time, timeout=timeout))
        delay_9 = TimerBehaviour(timer_duration=timer_delay, name="Delay 9")
        
        # Moving to X=2.0 while facing 45 degrees forces equal surge and sway
        diag_out = BasicActionBehaviour("Diagonal Out", move_global(x=fwd_dis, y=0.0, z=test_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        delay_10 = TimerBehaviour(timer_duration=timer_delay, name="Delay 10")
        
        diag_back = BasicActionBehaviour("Diagonal Back", move_global(x=0.0, y=0.0, z=test_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        delay_11 = TimerBehaviour(timer_duration=timer_delay, name="Delay 11")

        # =====================================================================
        # 11. Roll Test (+- 45 degrees)
        # =====================================================================
        roll_right = BasicActionBehaviour("Roll 45 Right", set_attitude(roll=math.pi/4, hold_time=hold_time, timeout=timeout))
        delay_12 = TimerBehaviour(timer_duration=timer_delay, name="Delay 12")

        roll_left = BasicActionBehaviour("Roll 45 Left", set_attitude(roll=-math.pi/4, hold_time=hold_time, timeout=timeout))
        delay_13 = TimerBehaviour(timer_duration=timer_delay, name="Delay 13")

        roll_level = BasicActionBehaviour("Roll Level", set_attitude(roll=0.0, hold_time=hold_time, timeout=timeout))
        delay_14 = TimerBehaviour(timer_duration=timer_delay, name="Delay 14")

        # =====================================================================
        # 12. Pitch Test (+- 45 degrees)
        # =====================================================================
        pitch_down = BasicActionBehaviour("Pitch 45 Down", set_attitude(pitch=math.pi/4, hold_time=hold_time, timeout=timeout))
        delay_15 = TimerBehaviour(timer_duration=timer_delay, name="Delay 15")

        pitch_up = BasicActionBehaviour("Pitch 45 Up", set_attitude(pitch=-math.pi/4, hold_time=hold_time, timeout=timeout))
        delay_16 = TimerBehaviour(timer_duration=timer_delay, name="Delay 16")

        pitch_level = BasicActionBehaviour("Pitch Level", set_attitude(pitch=0.0, hold_time=hold_time, timeout=timeout))
        delay_17 = TimerBehaviour(timer_duration=timer_delay, name="Delay 17")

        # =====================================================================
        # 13. Final Reset
        # =====================================================================
        turn_fwd_2 = BasicActionBehaviour("Face Forward Final", set_global_yaw(yaw_rad=0.0, hold_time=hold_time, timeout=timeout))
        delay_18 = TimerBehaviour(timer_duration=timer_delay, name="Delay 18")

        # =====================================================================
        # 14. Surface
        # =====================================================================
        surface = BasicActionBehaviour("Surface", set_depth(z=surface_depth, hold_time=hold_time, timeout=timeout))
        
        self.add_children([
            dive, delay_0,
            surge_fwd, delay_1,
            surge_bwd, delay_2,
            turn_90, delay_3,
            sway_right, delay_4,
            sway_left, delay_5,
            # turn_fwd_1, delay_6,
            # heave_up, delay_7,
            # heave_down, delay_8,
            turn_45, delay_9,
            diag_out, delay_10,
            diag_back, delay_11,
            # roll_right, delay_12,
            # roll_left, delay_13,
            # roll_level, delay_14,
            # pitch_down, delay_15,
            # pitch_up, delay_16,
            # pitch_level, delay_17,
            turn_fwd_2, delay_18,
            surface
        ])
