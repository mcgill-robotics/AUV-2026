# Python dependencies
import py_trees

# Planner dependencies
from ..robosub2026.gate_behaviours import create_style_rolling_flip_sequence


class TestStyleRollingFlip(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is a standalone test mission for the Style Rolling Flip (barrel roll) maneuver.
    It tests disabling attitude/planar controllers, applying open-loop roll torque while monitoring gyro integration,
    and re-enabling controllers to verify the automatic odometry and orientation snap.
    """

    def __init__(
        self,
        roll_torque: float = 15.0,
        target_degrees: float = 720.0,
        coast_degrees: float = 180.0,
        timeout_sec: float = 15.0,
    ):
        super().__init__("TestStyleRollingFlip", memory=True)

        flip_sequence = create_style_rolling_flip_sequence(
            roll_torque=roll_torque,
            target_degrees=target_degrees,
            coast_degrees=coast_degrees,
            timeout_sec=timeout_sec,
        )

        self.add_children([flip_sequence])
