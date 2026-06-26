import py_trees
from controls.goal_helpers import set_depth
import planner.missions.vision_behaviours as vision_behaviours
from .table_octagon_behaviours import GoAboveTable, CheckAboveTable
from ..mission_behaviour_components import BasicActionBehaviour

class TableOctagonTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Table & Octagon Task (Resupply).
    AUV surfaces in a 9ft octagon, picks up role-specific items from a table,
    and delivers them to baskets.
    Items: Nut/Plug (Survey & Repair) vs Pill/Bandage (Search & Rescue).
    """
    def __init__(self, **kwargs):
        super().__init__("Table & Octagon Task", memory=True)

        self.blackboard = self.attach_blackboard_client(name="OctagonTaskBlackboard")
        position_tolerance = kwargs.get('position_tolerance', 0.3)
        hold_time = kwargs.get('hold_time', 1.0)
        timeout = kwargs.get('timeout', 30.0)

        # 1. Go to a set depth to view the table better. The table is best seen at a shallow depth, since we get to
        # see the top of the table, whereas at a lower depth there is a lot of empty space
        #3 Go up to table 
        go_above_table = GoAboveTable(**kwargs)

        # 2. Surface inside Octagon
        # 3. Locate Resupply table and items
        # 4. Pick up role-specific items
        # 5. Deliver to correct basket
        # 6. Optional: Face correct icons and perform rotations
        # surface_action = BasicActionBehaviour(
        #     name="Surface in Octagon", 
        #     goal=set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        # )

        navigation_only = kwargs.get("navigation_only", True)
        ending_dive_depth = kwargs.get("ending_dive_depth", -0.5)

        if navigation_only:
            dive_back_down = BasicActionBehaviour(
                name="Dive Back Down (Navigation Only)",
                goal=set_depth(z=ending_dive_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
            )
            self.add_children([
                go_above_table,
                dive_back_down,
                py_trees.behaviours.Success(name="Navigation Subtasks Complete")
            ])
        else:
            # TODO: Add DropItemInBasket here when ready
            self.add_children([
                go_above_table,
                py_trees.behaviours.Success(name="Placeholder Table & Octagon Success")
            ])
