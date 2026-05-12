"""Math utilities for the motion package."""

import math
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw (rotation around Z) from a geometry_msgs Quaternion.

    Uses the standard ZYX Euler decomposition. Only yaw is returned
    since roll/pitch are handled by the attitude controller.

    Args:
        q: geometry_msgs/Quaternion

    Returns:
        Yaw angle in radians, range [-pi, pi].
    """
    r = Rotation.from_quat([q.x, q.y, q.z, q.w])
    euler = r.as_euler('ZYX', degrees=False)
    return euler[0]


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a geometry_msgs Quaternion from a pure yaw rotation.

    Args:
        yaw: Yaw angle in radians.

    Returns:
        geometry_msgs/Quaternion with roll=0, pitch=0.
    """
    r = Rotation.from_euler('Z', yaw, degrees=False)
    q_arr = r.as_quat()
    return Quaternion(x=q_arr[0], y=q_arr[1], z=q_arr[2], w=q_arr[3])


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi].

    Args:
        angle: Angle in radians.

    Returns:
        Normalized angle in [-pi, pi].
    """
    return math.remainder(angle, 2.0 * math.pi)
