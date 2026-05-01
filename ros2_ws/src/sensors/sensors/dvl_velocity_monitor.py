#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper

from dvl_msgs.msg import DVL
from sensors.health_monitor import HealthMonitor

class DVLVelocityMonitor(HealthMonitor):
    def __init__(self, node):
        super().__init__(node)
        self.node.declare_parameter("dvl_beam_count",Parameter.Type.INTEGER)
        self.node.declare_parameter("velocity_topic", Parameter.Type.STRING)
        
        self.dvl_beam_count = self.node.get_parameter("dvl_beam_count").get_parameter_value().integer_value
        self.velocity_topic = self.node.get_parameter("velocity_topic").get_parameter_value().string_value
        
        self.velocity_sub = self.node.create_subscription(
            DVL,
            self.velocity_topic,
            self.sensor_callback,
            self.qos
        )
        
        self.healthy_vel_status = (DiagnosticStatus.OK, "DVL velocity healthy")
        self._status_details:dict = {}
        self.vel_status: tuple[bytes, str] = self.healthy_vel_status

    def check_status(self, stat:DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        if self._status_details:
            for key, value in self._status_details.items():
                stat.add(key, value)
        stat.summary(*self.vel_status)
        return stat
    
    def sensor_callback(self, msg:DVL) -> None:
        self.last_update_time = self.node.get_clock().now()
        # Reset detail from previous callback — prevents stale entries persisting
        self._status_details = {}

        if not msg.velocity_valid:
            self.vel_status = (DiagnosticStatus.WARN, "DVL velocity invalid")
            return
        
        if msg.status == 1:
            self._status_details["⚠ Thermal warning"] = (
                "<b>DVL may be overheating.</b> Ensure DVL is "
                "underwater during operation or disable acoustics."
            )
            self.current_status = (DiagnosticStatus.WARN,"⚠ DVL OVERHEATING WARNING — monitor temperature")
        if msg.status > 1:
            self._status_details["Status code"] = f"<b>Unknown: {msg.status}</b>"
            self.vel_status = (DiagnosticStatus.WARN, f"Invalid DVL velocity status code")
            return
        
        if msg.altitude < 0:
            self._status_details["Altitude"] = (
                f"<b>{msg.altitude:.3f} m</b> — expected &ge; 0"
            )
            self.vel_status = (DiagnosticStatus.WARN, "DVL velocity altitude negative")
            return
        
        invalid_beams: list[str] = []
        for beam in msg.beams:
            if not beam.valid:
                invalid_beams.append(str(beam.id))
            else:
                self._status_details[f"Beam {beam.id}"] = "Invalid"
        valid_beam_count = self.dvl_beam_count - len(invalid_beams)
        if invalid_beams:
            self._status_details["Beam validity"] = (
                f"<b>{valid_beam_count}/{self.dvl_beam_count}</b> beams valid &nbsp;|&nbsp; "
                f"Invalid: <b>{', '.join(invalid_beams)}</b>"
            )
            self.current_status = (
                DiagnosticStatus.WARN,
                f"DVL has {len(invalid_beams)} invalid beam(s): {', '.join(invalid_beams)}",
            )
            return

        self._status_details["Beam validity"] = (
            f"<b>{self.dvl_beam_count}/{self.dvl_beam_count}</b> beams valid"
        )
        self._status_details["Altitude"] = f"{msg.altitude:.3f} m"
        self.current_status = (DiagnosticStatus.OK, "DVL velocity healthy")

        self.vel_status = self.healthy_vel_status

def main(args = None):
    rclpy.init(args=args)
    node:Node = rclpy.create_node("dvl_velocity_monitor")
    DVLVelocityMonitor(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == "__main__":
    main(args=sys.argv)