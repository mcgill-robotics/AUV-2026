"""Math utilities for the motion package."""

import math
from geometry_msgs.msg import Quaternion


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw (rotation around Z) from a geometry_msgs Quaternion.

    Uses the standard ZYX Euler decomposition. Only yaw is returned
    since roll/pitch are handled by the attitude controller.

    Args:
        q: geometry_msgs/Quaternion

    Returns:
        Yaw angle in radians, range [-pi, pi].
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a geometry_msgs Quaternion from a pure yaw rotation.

    Args:
        yaw: Yaw angle in radians.

    Returns:
        geometry_msgs/Quaternion with roll=0, pitch=0.
    """
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi].

    Args:
        angle: Angle in radians.

    Returns:
        Normalized angle in [-pi, pi].
    """
    return math.remainder(angle, 2.0 * math.pi)
