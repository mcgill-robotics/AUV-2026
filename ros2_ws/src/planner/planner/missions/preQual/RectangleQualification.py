import py_trees
import py_trees_ros
import rclpy
from controls import navigation_client
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from controls.goal_helpers import set_depth, set_global_yaw, move_global
from ...SensorsBehaviour import SensorsBehaviour
from ...BasicActionBehaviour import BasicActionBehaviour
import math

class RectangleQualificationMission(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the rectangle pre-qualification mission
    """
    def __init__(self, node):
        super().__init__("RectanglePrequalification", memory=True)
        # Build the full mission sequence
        # 1. Dive to -1.5m
        dive_leaf = BasicActionBehaviour(node, "Dive", set_depth(z=-1.5, tolerance=0.15, hold_time=2.0))
        
        # 2. Go to point where square begins (split this into three segments to accomodate for mcgill pool slant) 
        go_square_start_leaf = BasicActionBehaviour(node, "Move to rectangle start point", move_global(x=11.5, y=0.0, do_z=False, tolerance=0.5, hold_time=1.0))

        # 2.5 (Add a vision check? no rotation unless object in frame)

        ## Steps 3-7 are to move in a box around the marker for prequal
        # 3. Move right by a meter 
        square_bottom_left_leaf = BasicActionBehaviour(node, "Square: Bottom Left corner", move_global(x=11.5, y=-1.0, do_z=False, tolerance=0.5, hold_time=1.0))
        
        # 4. Move up by 3 meters and rotate 90 deg
        square_top_left_leaf = BasicActionBehaviour(node, "Square: Top Left corner", move_global(x=14.5, y=-1.0, yaw=math.pi/2, do_z=False, tolerance=0.5, hold_time=1.0))
        
        # 5. Move left by 2 meters and rotate 90 deg
        square_top_right_leaf = BasicActionBehaviour(node, "Square: Top Right corner", move_global(x=14.5, y=1.0, yaw=math.pi, do_z=False, tolerance=0.5, hold_time=1.0))
        
        # 6. Move down by 3 meters and rotate 90 deg
        square_bottom_right_leaf = BasicActionBehaviour(node, "Square: Bottom Right corner", move_global(x=11.5, y=1.0, yaw=math.pi * 3/2, do_z=False, tolerance=0.5, hold_time=1.0))

        # 7. Move right by a meter and rotate 90 deg
        # Same as 2 (cant re-use a child).
        go_square_start_leaf_2 = BasicActionBehaviour(node, "Move back to rectangle start point", move_global(x=11.5, y=0.0, yaw=0.0, do_z=False, tolerance=0.5, hold_time=1.0))

        
        # 8. Turn 180 degrees to look at the gate
        turn_leaf = BasicActionBehaviour(node, "Turn 180", set_global_yaw(yaw_rad=math.pi, tolerance=0.1, hold_time=1.0))
        
        # 9. Return to origin (X=0, Y=0) at the current depth
        return_leaf = BasicActionBehaviour(node, "Return to Origin", move_global(x=0.0, y=0.0, do_z=False, tolerance=0.5, hold_time=1.0))
        
        # 10. Ascend to surface
        ascend_leaf = BasicActionBehaviour(node, "Ascend to Surface", set_depth(z=0.0, tolerance=0.15, hold_time=1.0))
        
        self.add_children([dive_leaf, 
            go_square_start_leaf, 
            square_bottom_left_leaf,
            square_top_left_leaf, 
            square_top_right_leaf, 
            square_bottom_right_leaf,
            go_square_start_leaf_2,
            turn_leaf,
            return_leaf,
            ascend_leaf
            ])
