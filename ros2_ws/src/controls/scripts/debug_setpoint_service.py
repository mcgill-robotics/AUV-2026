#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data

import controls.utils as geometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from auv_msgs.srv import AUVSetpoint


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
        self.x_pub = self.create_publisher(Float64, '/controls/x_setpoint', qos)
        self.y_pub = self.create_publisher(Float64, '/controls/y_setpoint', qos)
        self.depth_pub = self.create_publisher(Float64, '/controls/depth_setpoint', qos)
        self.attitude_pub = self.create_publisher(Quaternion, '/controls/quaternion_setpoint', qos)
        self.setpoint_srv = self.create_service(AUVSetpoint, '/controls/debug_setpoint', self.setpoint_callback)
        
        self.setpoint: AUVSetpoint.Request = AUVSetpoint.Request()

    def setpoint_callback(self, req, res):
        self.setpoint = req

        self.x_pub.publish(Float64(data=req.x))
        self.y_pub.publish(Float64(data=req.y))
        self.depth_pub.publish(Float64(data=req.z))
        q = geometry.euler_to_quaternion(req.roll, req.pitch, req.yaw)
        self.attitude_pub.publish(q)
        res.success = True
        res.message = f"Setpoint updated to {req.x}, {req.y}, {req.z}, {req.roll}, {req.pitch}, {req.yaw}"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = DebugSetpointService()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()