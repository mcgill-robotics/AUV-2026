from matplotlib.pylab import det
import py_trees
import planner.missions.vision_behaviours as vision_behaviours
from planner.missions.robosub2026.bins_behaviors import FindBinStructure, GoNearObject, AlignCorrectBin
from planner.missions.mission_behaviour_components import BasicActionBehaviour
from controls.goal_helpers import move_global

class BinsTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Bins Task (Recon).
    AUV must locate bins on a 3D pipeline and drop markers into role-matching bins.
    Targets: Fire (Survey & Repair) vs Blood (Search & Rescue).
    """
    def __init__(self, **bins_params):
        super().__init__("Bins Task", memory=True)

        self.blackboard = self.attach_blackboard_client(name="BinsTaskBlackboard")
        # 0. Skip to bin structure
        skip_to_bin_structure = BasicActionBehaviour("SkipToBinStructure", goal=move_global(x=9.0, y=-7.0, yaw=0.0, z=-1.5))
        # 1. Search for and locate 3D pipeline/bins structure
        search_for_bin_structure = vision_behaviours.SearchSweepBehaviour(
            target_class="bin_structure" if not bins_params.get('force_fallback_search', False) else "bin",
            num_steps=bins_params.get('search_sweep_steps', 8),
            step_timeout=bins_params.get('search_sweep_step_timeout', 0.5))
        
        # 2. Go to bin structure
        if bins_params.get('force_fallback_search', False):
            go_near_bin_structure = GoNearObject(
                target_class="bin",
                target_distance=bins_params.get('bin_structure_distance', 2.0),
                height_offset=bins_params.get('go_above_bin_structure_height', 0.5)
            )
        else:
            go_near_bin_structure = FindBinStructure(
                target_distance=bins_params.get('bin_structure_distance', 2.0),
                height_offset=bins_params.get('go_above_bin_structure_height', 0.5)
            )

        # 3. Find the closest bin
        search_for_bins = vision_behaviours.SearchSweepBehaviour(
            target_class="bin",
            num_steps=bins_params.get('search_sweep_steps', 8),
            step_timeout=bins_params.get('search_sweep_step_timeout', 0.5)
        )

        # 4. Align with correct bin (try as many bins as needed until we have dropped both markers in the correct bins)
        align_correct_bin = AlignCorrectBin(bins_params)

        self.add_children([
            skip_to_bin_structure,
            search_for_bin_structure,
            go_near_bin_structure,
            search_for_bins,
            align_correct_bin,
        ])
