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

def quaternion_to_euler(q: Quaternion) -> tuple[float, float, float]:
    """Convert a geometry_msgs Quaternion to Euler angles (roll, pitch, yaw).

    Uses the standard ZYX Euler decomposition.

    Args:
        q: geometry_msgs/Quaternion

    Returns:
        Tuple of (roll, pitch, yaw) in radians.
    """
    r = Rotation.from_quat([q.x, q.y, q.z, q.w])
    euler = r.as_euler('ZYX', degrees=False)
    return euler[2], euler[1], euler[0]  # roll, pitch, yaw

def rotate_quaternion(q: Quaternion, roll_offset: float, pitch_offset: float, yaw_offset: float) -> Quaternion:
    """Rotate a quaternion by the specified offsets.

    Args:
        q: The original quaternion.
        roll_offset: Roll offset in radians.
        pitch_offset: Pitch offset in radians.
        yaw_offset: Yaw offset in radians.

    Returns:
        The rotated quaternion.
    """
    r = Rotation.from_quat([q.x, q.y, q.z, q.w])
    r = r * Rotation.from_euler('XYZ', [roll_offset, pitch_offset, yaw_offset], degrees=False)
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

def dot_product(q1: Quaternion, q2: Quaternion) -> float:
    """Compute the dot product of two quaternions. This is used to determine if two quaternions are in the same hemisphere, which is necessary for averaging quaternions."""
    return q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z

def compute_mean_orientation(orientations: list[Quaternion]) -> Quaternion:
    """Compute the mean orientation from a list of quaternions, as a naive average. This is not a true mean, but is a good approximation as long as the quaternions are close together."""
    mean_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    canonical_orientation = orientations[0]  # Use the first sample as a reference for hemisphere
    for sample in orientations:
        # correct for double cover issue by ensuring quaternions are in the same hemisphere
        if dot_product(canonical_orientation, sample) < 0:
            sample = Quaternion(x=-sample.x, y=-sample.y, z=-sample.z, w=-sample.w)
        mean_orientation.x += sample.x
        mean_orientation.y += sample.y
        mean_orientation.z += sample.z
        mean_orientation.w += sample.w
    n = len(orientations)
    mean_orientation.x /= n
    mean_orientation.y /= n
    mean_orientation.z /= n
    mean_orientation.w /= n
    return mean_orientation

def quaternion_distance(q1: Quaternion, q2: Quaternion) -> float:
    """Compute a naive distance between two quaternions as the Euclidean distance in quaternion space. This is not a true geodesic distance, but is sufficient for outlier rejection as long as the quaternions are close together."""
    distance = math.sqrt(
        (q1.x - q2.x) ** 2 +
        (q1.y - q2.y) ** 2 +
        (q1.z - q2.z) ** 2 +
        (q1.w - q2.w) ** 2
    )
    return distance

def is_quaternion_outlier(new_orientation: Quaternion, samples: list[Quaternion], rejection_threshold: float) -> bool:
    """Determine if a new quaternion orientation is an outlier compared to a list of samples."""
    if len(samples) == 0:
        return False  # No samples to compare against, so can't be an outlier

    # compute naive average orientation from samples (not a true mean, but good enough if all samples are close together)
    # TODO use geometric medoid instead
    mean_orientation = compute_mean_orientation(samples)

    # Compute distance between new orientation and mean orientation (also naive Euclidean distance in quaternion space, not a true geodesic distance)
    # TODO use angular distance instead
    distance = quaternion_distance(new_orientation, mean_orientation)
    
    return distance > rejection_threshold
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
    
    
def find_normal_from_quaternion(q: Quaternion) -> Vector2D:
    """Find the normal vector in the XY plane corresponding to a given quaternion orientation."""
    # Rotate the forward vector (1, 0, 0) by the quaternion to get the normal vector
    r = Rotation.from_quat([q.x, q.y, q.z, q.w])
    normal_vector = r.apply([1, 0, 0])
    return Vector2D(x=normal_vector[0], y=normal_vector[1])