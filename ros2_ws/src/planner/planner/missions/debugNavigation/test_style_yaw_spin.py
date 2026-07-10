# Python dependencies
import math
import py_trees

# Planner dependencies
from ..robosub2026.gate_behaviours import create_style_yaw_spin_sequence


class TestStyleYawSpin(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is a standalone test mission for the Style Yaw Spin maneuver.
    It executes clean stepped rotations (e.g., 3 steps of 120° = 360° spin, or 6 steps = 720° spin)
    to verify heading control and style point maneuvers in isolation.
    """

    def __init__(
        self,
        total_degrees: float = 360.0,
        num_steps: int = 3,
        angular_tolerance_deg: float = 8.0,
        hold_time_s: float = 0.5,
        timeout_s: float = 10.0,
    ):
        super().__init__("TestStyleYawSpin", memory=True)

        spin_sequence = create_style_yaw_spin_sequence(
            total_degrees=total_degrees,
            num_steps=num_steps,
            angular_tolerance_deg=angular_tolerance_deg,
            hold_time_s=hold_time_s,
            timeout_s=timeout_s,
        )

        self.add_children([spin_sequence])
