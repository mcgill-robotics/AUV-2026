import py_trees
import planner.missions.vision_behaviours as vision_behaviours
from planner.missions.robosub2026.bins_behaviors import FindBinStructure, ApproachObject, AlignCorrectBin
from planner.missions.mission_behaviour_components import BasicActionBehaviour
from controls.goal_helpers import set_depth

class BinsTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Bins Task (Recon).
    AUV must locate bins on a 3D pipeline and drop markers into role-matching bins.
    Targets: Fire (Survey & Repair) vs Blood (Search & Rescue).
    """
    def __init__(self, **bins_params):
        super().__init__("Bins Task", memory=True)

        self.blackboard = self.attach_blackboard_client(name="BinsTaskBlackboard")
        # 1. Search for and locate 3D pipeline/bins structure
        search_for_bin_structure = vision_behaviours.SearchSweepBehaviour(
            target_class="bin_structure" if not bins_params.get('force_fallback_search', False) else "bin",
            num_steps=bins_params['search_sweep_steps'],
            step_timeout=bins_params['search_sweep_step_timeout'])
        
        # 2. Initial depth setpoint
        initial_depth = BasicActionBehaviour(
            name="Initial Depth Setpoint",
            goal=set_depth(bins_params['initial_depth'], tolerance=bins_params['initial_depth_tolerance'], hold_time=0.0)
        )

        # 2. Go to bin structure
        if bins_params.get('force_fallback_search', False):
            go_near_bin_structure = ApproachObject(
                target_class="bin",
                bins_params=bins_params,
            )
        else:
            go_near_bin_structure = FindBinStructure(
                bins_params=bins_params
            )

        # 3. Find the closest bin
        search_for_bins = vision_behaviours.SearchSweepBehaviour(
            target_class="bin",
            num_steps=bins_params['search_sweep_steps'],
            step_timeout=bins_params['search_sweep_step_timeout']
        )

        # 4. Align with correct bin (try as many bins as needed until we have dropped both markers in the correct bins)
        align_correct_bin = AlignCorrectBin(bins_params)

        self.add_children([
            search_for_bin_structure,
            initial_depth,
            go_near_bin_structure,
            search_for_bins,
            align_correct_bin,
        ])
