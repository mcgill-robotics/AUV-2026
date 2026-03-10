"""Convenience functions for constructing AUVNavigate goals.

These helpers let BT nodes and other clients create goals in one line
without manually setting boolean flags. Example usage::

    from motion.goal_helpers import move_global, set_depth, stabilize

    goal = set_depth(z=-1.5)
    goal = move_global(x=2.0, y=3.0, z=-1.0, yaw=1.57)
    goal = stabilize(hold_time=5.0)
"""

from geometry_msgs.msg import Pose, Point, Quaternion
from auv_msgs.action import AUVNavigate
from motion.utils import quaternion_from_yaw

# Default tolerances used across all helpers
_DEFAULT_POS_TOL = 0.1     # meters
_DEFAULT_YAW_TOL = 0.175   # radians (~10 degrees)
_DEFAULT_HOLD = 2.0        # seconds
_DEFAULT_TIMEOUT = 30.0    # seconds


def _make_goal(
    target_pose: Pose,
    do_x: bool = False,
    do_y: bool = False,
    do_z: bool = False,
    do_yaw: bool = False,
    is_relative: bool = False,
    is_robot_centric: bool = False,
    position_tolerance: float = _DEFAULT_POS_TOL,
    yaw_tolerance: float = _DEFAULT_YAW_TOL,
    hold_time: float = _DEFAULT_HOLD,
    timeout: float = _DEFAULT_TIMEOUT,
) -> AUVNavigate.Goal:
    """Construct an AUVNavigate.Goal with all fields set."""
    # TODO: Create and return an AUVNavigate.Goal with all fields populated
    pass


# ─────────────────────────────────────────────────────────────────────────────
# Global Navigation (Absolute Pool Frame)
# ─────────────────────────────────────────────────────────────────────────────

def move_to_pose(
    pose: Pose,
    tolerance: float = _DEFAULT_POS_TOL,
    yaw_tolerance: float = _DEFAULT_YAW_TOL,
    hold_time: float = _DEFAULT_HOLD,
    timeout: float = _DEFAULT_TIMEOUT,
) -> AUVNavigate.Goal:
    """Move to an exact pose (all DOFs). Pass a VisionObject.pose directly.

    Args:
        pose: Target pose in pool frame.
        tolerance: Position convergence threshold in meters.
        yaw_tolerance: Yaw convergence threshold in radians.
        hold_time: Seconds to hold within tolerance before SUCCESS.
        timeout: Seconds before FAILURE (0 = no timeout).
    """
    # TODO
    pass


def move_global(
    x: float,
    y: float,
    z: float,
    yaw: float = None,
    tolerance: float = _DEFAULT_POS_TOL,
    yaw_tolerance: float = _DEFAULT_YAW_TOL,
    hold_time: float = _DEFAULT_HOLD,
    timeout: float = _DEFAULT_TIMEOUT,
) -> AUVNavigate.Goal:
    """Move to absolute XYZ in pool frame, optionally setting yaw.

    Args:
        x: Target X position in meters (pool frame).
        y: Target Y position in meters (pool frame).
        z: Target Z position in meters (negative = below surface).
        yaw: Target yaw in radians. None = don't control yaw.
        tolerance: Position convergence threshold in meters.
        yaw_tolerance: Yaw convergence threshold in radians.
        hold_time: Seconds to hold within tolerance before SUCCESS.
        timeout: Seconds before FAILURE (0 = no timeout).
    """
    # TODO
    pass


def set_depth(
    z: float,
    tolerance: float = _DEFAULT_POS_TOL,
    hold_time: float = _DEFAULT_HOLD,
    timeout: float = _DEFAULT_TIMEOUT,
) -> AUVNavigate.Goal:
    """Set the AUV depth (Z only). Other DOFs are unaffected.

    Args:
        z: Target depth in meters (negative = below surface).
        tolerance: Depth convergence threshold in meters.
        hold_time: Seconds to hold within tolerance before SUCCESS.
        timeout: Seconds before FAILURE (0 = no timeout).
    """
    # TODO
    pass


def set_global_yaw(
    yaw_rad: float,
    tolerance: float = _DEFAULT_YAW_TOL,
    hold_time: float = _DEFAULT_HOLD,
    timeout: float = _DEFAULT_TIMEOUT,
) -> AUVNavigate.Goal:
    """Rotate to an absolute yaw heading. Position is unaffected.

    Args:
        yaw_rad: Target yaw in radians.
        tolerance: Yaw convergence threshold in radians.
        hold_time: Seconds to hold within tolerance before SUCCESS.
        timeout: Seconds before FAILURE (0 = no timeout).
    """
    # TODO
    pass


# ─────────────────────────────────────────────────────────────────────────────
# Relative Navigation (Delta Movements)
# ─────────────────────────────────────────────────────────────────────────────

def move_robot_centric(
    forward: float = 0.0,
    sway: float = 0.0,
    heave: float = 0.0,
    dyaw: float = 0.0,
    tolerance: float = _DEFAULT_POS_TOL,
    yaw_tolerance: float = _DEFAULT_YAW_TOL,
    hold_time: float = _DEFAULT_HOLD,
    timeout: float = _DEFAULT_TIMEOUT,
) -> AUVNavigate.Goal:
    """Translate and/or rotate relative to the robot's body frame.

    This is the primary helper for teleop and robot-centric BT moves.
    All offsets are in the robot's body frame (forward/sway/heave).

    Args:
        forward: Meters forward (+X body, camera direction).
        sway: Meters left (+Y body).
        heave: Meters up (+Z body).
        dyaw: Radians to turn (positive = left).
        tolerance: Position convergence threshold in meters.
        yaw_tolerance: Yaw convergence threshold in radians.
        hold_time: Seconds to hold within tolerance before SUCCESS.
        timeout: Seconds before FAILURE (0 = no timeout).
    """
    # TODO: Auto-detect which DOFs to enable based on non-zero values
    pass


def translate_field_centric(
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    tolerance: float = _DEFAULT_POS_TOL,
    hold_time: float = _DEFAULT_HOLD,
    timeout: float = _DEFAULT_TIMEOUT,
) -> AUVNavigate.Goal:
    """Move by a delta in the pool frame regardless of heading.

    Args:
        dx: Meters in pool X direction.
        dy: Meters in pool Y direction.
        dz: Meters in pool Z direction (negative = descend).
        tolerance: Position convergence threshold in meters.
        hold_time: Seconds to hold within tolerance before SUCCESS.
        timeout: Seconds before FAILURE (0 = no timeout).
    """
    # TODO
    pass


def rotate_relative(
    dyaw_rad: float,
    tolerance: float = _DEFAULT_YAW_TOL,
    hold_time: float = _DEFAULT_HOLD,
    timeout: float = _DEFAULT_TIMEOUT,
) -> AUVNavigate.Goal:
    """Turn left/right by a delta from the current heading.

    Args:
        dyaw_rad: Radians to turn (positive = left).
        tolerance: Yaw convergence threshold in radians.
        hold_time: Seconds to hold within tolerance before SUCCESS.
        timeout: Seconds before FAILURE (0 = no timeout).
    """
    # TODO
    pass


# ─────────────────────────────────────────────────────────────────────────────
# Hold / Stabilize
# ─────────────────────────────────────────────────────────────────────────────

def stabilize(
    hold_time: float = _DEFAULT_HOLD,
    tolerance: float = _DEFAULT_POS_TOL,
    yaw_tolerance: float = _DEFAULT_YAW_TOL,
    timeout: float = 0.0,
) -> AUVNavigate.Goal:
    """Hold the current pose. Relative with zero offset = current pose.

    Use a very large hold_time (e.g. 999999) to freeze indefinitely.
    Default timeout=0 means no timeout (hold until preempted).

    Args:
        hold_time: Seconds to hold within tolerance before SUCCESS.
        tolerance: Position convergence threshold in meters.
        yaw_tolerance: Yaw convergence threshold in radians.
        timeout: Seconds before FAILURE (0 = no timeout).
    """
    # TODO
    pass
