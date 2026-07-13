import py_trees
from controls.goal_helpers import set_depth
import planner.missions.vision_behaviours as vision_behaviours
from .table_octagon_behaviours import GoAboveTable, TableCleaning, LookAndSpin
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
        go_above_table = GoAboveTable(**kwargs)

        # 2. Clean up tasks
        clean_up_tasks = TableCleaning(**kwargs)

        # 3 Look at the corresponding image and yaw in front of it
        look_and_yaw = LookAndSpin(**kwargs)

        navigation_only = kwargs.get("navigation_only", True)
        ending_dive_depth = kwargs.get("ending_dive_depth", -0.5)

    
        dive_back_down = BasicActionBehaviour(
            name="Dive Back Down (Navigation Only)",
            goal=set_depth(z=ending_dive_depth, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        )
        self.add_children([
            clean_up_tasks,
            
            py_trees.behaviours.Success(name="Navigation Subtasks Complete")
        ])
        
        # if not navigation_only:
        #     # TODO: Add DropItemInBasket here when ready
        #     self.add_children([
        #         clean_up_tasks,
        #         look_and_yaw,
        #         py_trees.behaviours.Success(name="Placeholder Table & Octagon Success")
        #     ])
