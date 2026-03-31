import py_trees
import py_trees_ros
import rclpy
from controls import navigation_client
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from controls.goal_helpers import set_depth, set_global_yaw, move_global
from ...SensorsBehaviour import SensorsBehaviour
from ...utils.BasicActionBehaviour import BasicActionBehaviour
from ...utils.MissionChoiceCheckBehaviour import MissionChoiceCheckBehaviour
from ...utils.MissionCompleteBehaviour import MissionCompleteBehaviour
from ...utils.TimerBehaviour import TimerBehaviour
import math

class RectangleQualificationMission(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the rectangle pre-qualification mission
    """
    def __init__(self, node):
        super().__init__("RectanglePrequalification", memory=True)
        self.node = node
        
        # Get the general parameters from the configs that were declared in root of Behaviour Tree
        yaw_tolerance = self.node.pre_qual_yaw_tolerance
        position_tolerance = self.node.pre_qual_positional_tolerance
        hold_time = self.node.pre_qual_hold_time
        timeout = self.node.pre_qual_timeout

        # 0 Check if user input the desired mission choce
        """
        1: Orbit Prequal
        2: Rectangle Prequal
        3: Basic Move forward
        4: Basic Dive
        """
        mission_choice_check = MissionChoiceCheckBehaviour(name="RectanglePrequalUserCheck", choice=2)

        # 1 Wait for 10 seconds before starting the mission
        timer = TimerBehaviour(node=node, timer=10.0, name="Orbit Prequal Timer")

        # Build the full mission sequence
        # 2. Dive to -1.5m
        dive_leaf = BasicActionBehaviour(node, "Dive", set_depth(z=-1.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 3. Go to point where square begins (split this into three segments to accomodate for mcgill pool slant) 
        go_square_start_leaf = BasicActionBehaviour(node, "Move to rectangle start point", move_global(x=11.5, y=0.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 3.5 (Add a vision check? no rotation unless object in frame)

        ## Steps 3-7 are to move in a box around the marker for prequal
        # 4. Move right by a meter 
        square_bottom_left_leaf = BasicActionBehaviour(node, "Square: Bottom Left corner", move_global(x=11.5, y=-1.0, do_z=False, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 5. Move up by 3 meters and rotate 90 deg
        square_top_left_leaf = BasicActionBehaviour(node, "Square: Top Left corner", move_global(x=14.5, y=-1.0, yaw=math.pi/2, do_z=False, tolerance=position_tolerance, yaw_tolerance=yaw_tolerance,hold_time=hold_time, timeout=timeout))
        
        # 6. Move left by 2 meters and rotate 90 deg
        square_top_right_leaf = BasicActionBehaviour(node, "Square: Top Right corner", move_global(x=14.5, y=1.0, yaw=math.pi, do_z=False, tolerance=position_tolerance, yaw_tolerance=yaw_tolerance,hold_time=hold_time, timeout=timeout))
        
        # 7. Move down by 3 meters and rotate 90 deg
        square_bottom_right_leaf = BasicActionBehaviour(node, "Square: Bottom Right corner", move_global(x=11.5, y=1.0, yaw=math.pi * 3/2, do_z=False, tolerance=position_tolerance, yaw_tolerance=yaw_tolerance,hold_time=hold_time, timeout=timeout))

        # 8. Move right by a meter and rotate 90 deg
        # Same as 2 (cant re-use a child).
        go_square_start_leaf_2 = BasicActionBehaviour(node, "Move back to rectangle start point", move_global(x=11.5, y=0.0, yaw=0.0, do_z=False, tolerance=position_tolerance, yaw_tolerance=yaw_tolerance,hold_time=hold_time, timeout=timeout))

        
        # 9. Turn 180 degrees to look at the gate
        turn_leaf = BasicActionBehaviour(node, "Turn 180", set_global_yaw(yaw_rad=math.pi, tolerance=yaw_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 10. Return to origin (X=0, Y=0) at the current depth
        return_leaf = BasicActionBehaviour(node, "Return to Origin", move_global(x=0.0, y=0.0, do_z=False, tolerance=position_tolerance,hold_time=hold_time, timeout=timeout))
        
        # 11. Ascend to surface
        ascend_leaf = BasicActionBehaviour(node, "Ascend to Surface", set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        
        # 12. Reset the user mission choice to allow for new mission to be selected
        mission_choice_reset = MissionCompleteBehaviour(node, "Completed Rectangle Prequal")

        self.add_children([mission_choice_check,
            timer,
            dive_leaf, 
            go_square_start_leaf, 
            square_bottom_left_leaf,
            square_top_left_leaf, 
            square_top_right_leaf, 
            square_bottom_right_leaf,
            go_square_start_leaf_2,
            turn_leaf,
            return_leaf,
            ascend_leaf,
            mission_choice_reset
            ])
