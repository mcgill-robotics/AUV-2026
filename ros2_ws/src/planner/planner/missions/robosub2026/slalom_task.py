import py_trees
from controls.goal_helpers import set_depth
from ..mission_behaviour_components import BasicActionBehaviour
from ..vision_behaviours import SearchSweepBehaviour, ScanBehaviour
from .slalom_behaviours import MatchPipesBehaviour, NavigateToGapBehaviour


class SlalomTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Slalom Task (Avoid Debris).

    The AUV navigates through 3 layers of vertical pipes. Each layer has
    1 red pipe in the center and 2 white pipes on either side. The AUV must
    pass on the same side of the red pipe as the gate divider side it chose.

    Each layer is handled by a sub-sequence visible in the tree viewer:
        SearchSweep → Scan Pipes → Match Pipes → Navigate To Gap

    Params:
        num_layers: Number of pipe layers to navigate (default 3).
        gate_side: Which side of the red pipe to pass ("left" or "right").
        scan_angle_deg: Angle to scan left/right from center during the scan phase.
        scan_pause_time: Seconds to pause after each scan rotation for vision.
        collinearity_threshold: Max perpendicular distance (m) for pipe triplet matching.
        min_forward_dist: Min forward distance (m) to consider a pipe as "in front".
    """
    def __init__(
        self,
        position_tolerance: float,
        hold_time: float,
        timeout: float,
        num_layers: int = 3,
        gate_side: str = "right",
        scan_angle_deg: float = 60.0,
        scan_pause_time: float = 1.0,
        collinearity_threshold: float = 0.5,
        min_forward_dist: float = 0.5,
    ):
        super().__init__("Slalom Task", memory=True)

        self.add_child(
            BasicActionBehaviour(
                name="Initial Dive (-1.0m)",
                goal=set_depth(z=-1.0, timeout=30.0),
            )
        )

        for i in range(num_layers):
            layer = py_trees.composites.Sequence(f"Slalom Layer {i + 1}", memory=True)
            children = []

            # Only do a full 360 search for the very first layer
            if i == 0:
                children.append(
                    SearchSweepBehaviour(
                        target_class="red_pipe",
                        num_steps=5,
                        max_attempts=2,
                        step_timeout=scan_pause_time,
                        clockwise=False,
                        look_at_on_success=True,
                        name=f"Search Red Pipe L{i + 1}",
                    )
                )

            children.extend([
                ScanBehaviour(
                    scan_angle_deg=scan_angle_deg,
                    pause_time=scan_pause_time,
                    name=f"Scan Pipes L{i + 1}",
                ),
                MatchPipesBehaviour(
                    gate_side=gate_side,
                    collinearity_threshold=collinearity_threshold,
                    min_forward_dist=min_forward_dist,
                    name=f"Match Pipes L{i + 1}",
                ),
                NavigateToGapBehaviour(
                    adjust_depth=(i == 0),
                    name=f"Navigate To Gap L{i + 1}",
                ),
            ])
            layer.add_children(children)
            self.add_child(layer)
