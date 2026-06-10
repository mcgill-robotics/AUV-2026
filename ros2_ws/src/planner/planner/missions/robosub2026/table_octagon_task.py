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
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Table & Octagon Task", memory=True)

        self.blackboard = self.attach_blackboard_client(name="OctagonTaskBlackboard")

        # 1. Go to a set depth to view the table better. The table is best seen at a shallow depth, since we get to
        # see the top of the table, whereas at a lower depth there is a lot of empty space
        go_shallow_depth = BasicActionBehaviour(
            name="Octagon: Go Shallow Depth",
            goal=set_depth(z=-0.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        )

        # 2. Look for the table 
        search_for_table = vision_behaviours.SearchSweepBehaviour(
            target_class="table",
            num_steps=5,
            step_timeout=0.5)
        

        #3 Go up to table 
        go_above_table = GoAboveTable(
            position_tolerance=position_tolerance,
            hold_time=hold_time,
            timeout=timeout
        )

        #4 Surface inside Octagon and align with table while at the surface 

        #4 Go above the table, with the more accurate position of the table 
        go_above_table_1m_away.add_child(go_above_table)

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
