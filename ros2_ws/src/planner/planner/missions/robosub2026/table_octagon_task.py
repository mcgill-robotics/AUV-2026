import py_trees
from controls.goal_helpers import move_robot_centric
from ..mission_behaviour_components import BasicActionBehaviour

class TableOctagonTask(py_trees.composites.Sequence):
    """
    Pure task sequence for the RoboSub 2026 Table & Octagon.
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Table & Octagon Task", memory=True)

        # 1. Search for Table (Placeholder)
        # TODO: Add vision alignment here
        
        # 2. Surface inside the Octagon (Placeholder)
        surface_action = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=move_robot_centric(forward=0.0, do_z=True, z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        )

        self.add_children([
            surface_action
        ])
