import py_trees
from controls.goal_helpers import set_depth
from ..mission_behaviour_components import BasicActionBehaviour

class TableOctagonTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Table & Octagon Task (Resupply).
    AUV surfaces in a 9ft octagon, picks up role-specific items from a table,
    and delivers them to baskets.
    Items: Nut/Plug (Survey & Repair) vs Pill/Bandage (Search & Rescue).
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Table & Octagon Task", memory=True)
        # TODO: Implement Table & Octagon Task
        # 1. Navigate to Octagon (Acoustic pinger)
        # 2. Surface inside Octagon
        # 3. Locate Resupply table and items
        # 4. Pick up role-specific items
        # 5. Deliver to correct basket
        # 6. Optional: Face correct icons and perform rotations
        surface_action = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        )

        self.add_children([
            surface_action,
            py_trees.behaviours.Success(name="Placeholder Table & Octagon Success")
        ])
