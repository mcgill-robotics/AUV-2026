import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data


class DebugSetpointService(Node):
    """
    Node to set all setpoints as a service. For conviennce, attitude is set in euler angles (roll, pitch, yaw) in degrees.
    """
    def __init__(self):
        super().__init__('debug_setpoint_service')

        # QoS: Always use most recent readings, reliable, volatile durability
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.setpoint_srv = self.create_service(AUVSetpoint, '/controls/debug_setpoint', self.setpoint_callback, qos)

    def setpoint_callback(self, req, res):
        res.success = True
        res.message = f"Setpoint updated to {req.x}, {req.y}, {req.z}, {req.roll}, {req.pitch}, {req.yaw}"
        return res