from rclpy.parameter import Parameter
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper

from dvl_msgs.msg import DVL
from sensors.health_monitor import HealthMonitor

from typing import Tuple


class DVLVelocityMonitor(HealthMonitor):
    def __init__(self, node):
        super().__init__(node)
        self.node.declare_parameter("velocity_topic", Parameter.Type.STRING)
        
        self.velocity_topic = self.node.get_parameter("velocity_topic").get_parameter_value().string_value
        
        self.velocity_sub = self.node.create_subscription(
            DVL,
            self.velocity_topic,
            self.sensor_callback,
            self.qos
        )
        
        self.healthy_vel_status = (DiagnosticStatus.OK, "DVL velocity healthy")
        self.vel_status: tuple[bytes, str] = self.healthy_vel_status

    def check_status(self, stat:DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        stat.summary(*self.vel_status)
        return stat
    
    def sensor_callback(self, msg:DVL) -> None:
        self.last_update_time = self.node.get_clock().now()
        if not msg.velocity_valid:
            self.velocity_status = (DiagnosticStatus.WARN, "DVL velocity invalid")
            return
        if msg.status > 0:
            self.velocity_status = (DiagnosticStatus.ERROR, f"Invalid DVL velocity status code: {msg.status}. Device may be overheating")
            return
        self.velocity_status = self.healthy_vel_status
        
            