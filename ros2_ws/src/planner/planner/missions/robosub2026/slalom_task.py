import math
import py_trees
from controls.goal_helpers import set_depth, translate_field_centric
from ..mission_behaviour_components import BasicActionBehaviour
from ..vision_behaviours import SearchSweepBehaviour
from .slalom_behaviours import SlalomLayer


class SlalomTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Slalom Task (Avoid Debris).

    The AUV navigates through 3 layers of vertical pipes. Each layer has
    1 red pipe in the center and 2 white pipes on either side. The AUV must
    pass on the same side of the red pipe as the gate divider side it chose.

    Each layer is handled by a sub-sequence visible in the tree viewer:
        SearchSweep -> Scan Pipes -> Match Pipes -> Navigate To Gap

    Params:
        num_layers: Number of pipe layers to navigate (default 3).
        gate_side: Fallback default for which side of the red pipe to pass ("left" or "right").
                   Overridden at runtime by /gate/selected_side on the blackboard (set by GateTask).
        scan_angle_deg: (deg) Angle to scan left/right from center during the scan phase.
        scan_pause_time: (s) Seconds to pause after each scan rotation for vision.
        collinearity_threshold: (m) Max perpendicular distance for pipe triplet matching.
        min_forward_dist: (m) Min forward distance to consider a pipe as "in front".
        layer_distance: (m) Expected distance between pipe layers (used for failsafe).
        initial_depth: (m) Depth for the initial dive before slalom (negative = below surface).
        position_tolerance: (m) Position convergence threshold for gap navigation.
        angular_tolerance: (rad) Yaw convergence threshold for gap navigation.
        hold_time: (s) Seconds to hold within tolerance before declaring SUCCESS.
        timeout: (s) Seconds before gap navigation declares FAILURE.
        scan_angular_tolerance_rad: (rad) Yaw tolerance for scan/sweep turn steps.
        scan_hold_time: (s) Hold time before scan turn step SUCCESS.
        scan_timeout: (s) Timeout before scan turn step FAILURE.
    """
    def __init__(
        self,
        num_layers: int = 3,
        gate_side: str = "right",
        yaw_inward_offset_deg: float = 10.0,
        scan_angle_deg: float = 60.0,
        scan_pause_time: float = 1.0,
        collinearity_threshold: float = 0.5,
        min_forward_dist: float = 0.5,
        layer_distance: float = 2.0,
        initial_depth: float = -1.0,
        position_tolerance: float = 0.3,
        angular_tolerance_rad: float = 0.3,
        hold_time: float = 0.5,
        timeout: float = 45.0,
        scan_angular_tolerance_rad: float = math.radians(30.0),
        scan_hold_time: float = 0.1,
        scan_timeout: float = 30.0,
    ):
        super().__init__("Slalom Task", memory=True)

        self.add_child(
            BasicActionBehaviour(
                name=f"Initial Dive ({initial_depth}m)",
                goal=set_depth(z=initial_depth, timeout=30.0),
            )
        )

        # Strategy Selector: Try nominal slalom, if completely lost initially, fallback to full length drive
        strategy_selector = py_trees.composites.Selector("Slalom Strategy", memory=True)

        # --- NOMINAL SLALOM EXECUTION ---
        nominal_execution = py_trees.composites.Sequence("Nominal Execution", memory=True)

        # 1. Initial Search: Try finding a red pipe, fallback to white pipe
        initial_search = py_trees.composites.Selector("Initial Target Search", memory=True)
        initial_search.add_children([
            SearchSweepBehaviour(
                target_class="red_pipe",
                num_steps=5,
                max_attempts=2,
                step_timeout=scan_pause_time,
                clockwise=False,
                look_at_on_success=True,
                angular_tolerance_rad=scan_angular_tolerance_rad,
                turn_hold_time_s=scan_hold_time,
                turn_timeout_s=scan_timeout,
                name="Search Red Pipe (Primary)",
            ),
            SearchSweepBehaviour(
                target_class="white_pipe",
                num_steps=5,
                max_attempts=2,
                step_timeout=scan_pause_time,
                clockwise=False,
                look_at_on_success=True,
                angular_tolerance_rad=scan_angular_tolerance_rad,
                turn_hold_time_s=scan_hold_time,
                turn_timeout_s=scan_timeout,
                name="Search White Pipe (Fallback)",
            ),
        ])
        nominal_execution.add_child(initial_search)

        # 2. Add each layer as a self-recovering sequence
        for i in range(num_layers):
            layer = SlalomLayer(
                layer_num=i + 1,
                gate_side=gate_side,
                yaw_inward_offset_deg=yaw_inward_offset_deg,
                scan_angle_deg=scan_angle_deg,
                scan_pause_time=scan_pause_time,
                collinearity_threshold=collinearity_threshold,
                min_forward_dist=min_forward_dist,
                layer_distance=layer_distance,
                adjust_depth=(i == 0),
                position_tolerance=position_tolerance,
                angular_tolerance_rad=angular_tolerance_rad,
                hold_time=hold_time,
                timeout=timeout,
                scan_angular_tolerance_rad=scan_angular_tolerance_rad,
                scan_hold_time=scan_hold_time,
                scan_timeout=scan_timeout,
            )
            nominal_execution.add_child(layer)

        # --- TOTAL FAILSAFE ---
        total_failsafe = BasicActionBehaviour(
            name="Total Slalom Failsafe (Blind Drive Field-X)",
            goal=translate_field_centric(
                dx=num_layers * layer_distance, 
                tolerance=position_tolerance,
                hold_time=hold_time,
                timeout=timeout
            ),
        )

        strategy_selector.add_children([nominal_execution, total_failsafe])
        self.add_child(strategy_selector)
