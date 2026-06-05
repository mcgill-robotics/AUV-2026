"""Math utilities for the motion package."""

import math
from geometry_msgs.msg import Point, Quaternion
from scipy.spatial.transform import Rotation
from dataclasses import dataclass

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

@dataclass
class Vector2D:
    x: float
    y: float

    def __add__(self, other: "Vector2D") -> "Vector2D":
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector2D") -> "Vector2D":
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vector2D":
        return Vector2D(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> "Vector2D":
        return self.__mul__(scalar)   # supports: 3.0 * v

    def __truediv__(self, scalar: float) -> "Vector2D":
        return Vector2D(self.x / scalar, self.y / scalar)

    def __neg__(self) -> "Vector2D":
        return Vector2D(-self.x, -self.y)

    def dot(self, other: "Vector2D") -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: "Vector2D") -> float:
        return self.y * other.x - self.x * other.y

    def __str__(self) -> str:
        return f"Vector({self.x:.2f}, {self.y:.2f})"
    
    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self) -> "Vector2D":
        n = self.norm()
        if n == 0:
            raise ValueError("Cannot normalize a zero vector")
        return self / n
    @staticmethod
    def from_point(point: Point) -> "Vector2D":
        return Vector2D(x=point.x, y=point.y)