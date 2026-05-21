from matplotlib.pylab import det
import py_trees
import planner.missions.vision_behaviours as vision_behaviours
import planner.missions.mission_behaviour_components as basic_behaviours
from controls.goal_helpers import move_global
from planner.missions.robosub2026.bins_behaviors import GoAboveClosestBin, GoNearBinStructure

class BinsTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Bins Task (Recon).
    AUV must locate bins on a 3D pipeline and drop markers into role-matching bins.
    Targets: Fire (Survey & Repair) vs Blood (Search & Rescue).
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Bins Task", memory=True)

        self.blackboard = self.attach_blackboard_client(name="BinsTaskBlackboard")
        # TODO: Implement Bins Task
        # 0. For testing purposes, skip the first few tasks and go close enough to bin structure to detect it
        skip_to_bin_structure = basic_behaviours.BasicActionBehaviour(
            name="Skip to Bin Structure",
            goal=move_global(5.0, 0.0, -1.0))
        # 1. Search for and locate 3D pipeline/bins structure
        search_for_bin_structure = vision_behaviours.SearchSweepBehaviour(
            target_class="bin_structure",
            num_steps=8,
            max_attempts=2,
            step_timeout=0.5)
        
        # 2. Go to bin structure
        go_near_bin_structure = GoNearBinStructure()

        # 3. Find the closest bin
        search_for_bins = vision_behaviours.SearchSweepBehaviour(
            target_class="bin",
            num_steps=8,
            max_attempts=2,
            step_timeout=0.5)
        
        # 4. Go over that bin and use downcam to line up
        go_to_closest_bin = GoAboveClosestBin()

        # 5. Drop marker in bin (not implemented yet, just placeholder success behavior for now)
        drop_marker = py_trees.behaviours.Success(name="Placeholder Drop Marker Success")

        # 6. Optional: Detect light and turn off via magnetic detector

        self.add_children([
            skip_to_bin_structure,
            search_for_bin_structure,
            go_near_bin_structure,
            search_for_bins,
            go_to_closest_bin,
            drop_marker
        ])
