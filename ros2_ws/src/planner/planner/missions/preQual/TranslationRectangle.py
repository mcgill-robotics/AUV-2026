import py_trees
import py_trees_ros
import rclpy
from controls import navigation_client
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from controls.goal_helpers import set_depth, move_global
from ...SensorsBehaviour import SensorsBehaviour
from ...utils.BasicActionBehaviour import BasicActionBehaviour
from ...utils.MissionChoiceCheckBehaviour import MissionChoiceCheckBehaviour
from ...utils.MissionCompleteBehaviour import MissionCompleteBehaviour
from ...utils.TimerBehaviour import TimerBehaviour

class TranslationRectangleMission(py_trees.composites.Sequence):
    """
    Same rectangle path as the RectangleQualification mission but with
    NO yaw commands - the AUV only translates between waypoints.
    Useful for pool-testing the planar controller without attitude changes.
    """
    def __init__(self, node):
        super().__init__("TranslationRectangle", memory=True)
        self.node = node
        
        # Get the general parameters from the configs that were declared in root of Behaviour Tree
        position_tolerance = self.node.pre_qual_positional_tolerance
        hold_time = self.node.pre_qual_hold_time
        timeout = self.node.pre_qual_timeout

        # 0 Check if user chose this mission
        mission_choice_check = MissionChoiceCheckBehaviour(name="TranslationRectangleCheck", choice=6)

        # 1 Wait for 10 seconds before starting the mission
        timer = TimerBehaviour(node=node, timer=10.0, name="Translation Rectangle Timer")

        # 2. Dive to -1.5m
        dive_leaf = BasicActionBehaviour(node, "Dive", set_depth(z=-1.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 3. Move past the gate (5m forward)
        past_gate = BasicActionBehaviour(node, "Move past gate", move_global(x=5.0, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 4. Ascend to -0.5m (clear the gate)
        ascend_past_gate = BasicActionBehaviour(node, "Ascend past gate", set_depth(z=-0.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 5. Go to rectangle start point
        go_start = BasicActionBehaviour(node, "Move to rectangle start", move_global(x=11.5, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 4. Bottom left corner
        bottom_left = BasicActionBehaviour(node, "Bottom Left", move_global(x=11.5, y=-1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 5. Top left corner
        top_left = BasicActionBehaviour(node, "Top Left", move_global(x=14.5, y=-1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 6. Top right corner
        top_right = BasicActionBehaviour(node, "Top Right", move_global(x=14.5, y=1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 7. Bottom right corner
        bottom_right = BasicActionBehaviour(node, "Bottom Right", move_global(x=11.5, y=1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 8. Back to start
        go_start_2 = BasicActionBehaviour(node, "Back to rectangle start", move_global(x=11.5, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # Move back to gate area (5m)
        back_to_gate = BasicActionBehaviour(node, "Back to gate", move_global(x=5.0, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # Descend to -1.5m to go back under the gate
        descend_at_gate = BasicActionBehaviour(node, "Descend at gate", set_depth(z=-1.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # Return to origin
        return_leaf = BasicActionBehaviour(node, "Return to Origin", move_global(x=0.0, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 10. Ascend to surface
        ascend_leaf = BasicActionBehaviour(node, "Ascend to Surface", set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 11. Reset mission choice
        mission_choice_reset = MissionCompleteBehaviour(node, "Completed Translation Rectangle")

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
