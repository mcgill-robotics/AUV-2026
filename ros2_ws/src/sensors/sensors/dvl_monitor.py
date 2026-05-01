#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.time import Time

from dvl_msgs.msg import DVL,DVLDR
from nav_msgs.msg import Odometry
from sensors.health_monitor import HealthMonitor

class DVLMonitor(HealthMonitor):
    BEAMIDX_TO_NAME = {
        0: "Back Left",
        1: "Forward Left",
        2: "Forward Right",
        3: "Back Right",
    }
    def __init__(self, node):
        super().__init__(node)
        # Declare DVL-specific parameters
        self.node.declare_parameter("dvl_beam_count",Parameter.Type.INTEGER)
        self.node.declare_parameter("velocity_topic", Parameter.Type.STRING)
        self.node.declare_parameter("dead_reckoning_topic", Parameter.Type.STRING)
        self.node.declare_parameter("odometry_topic", Parameter.Type.STRING)
        # Assign parameters to instance variables
        self.dvl_beam_count = self.node.get_parameter("dvl_beam_count").get_parameter_value().integer_value
        velocity_topic = self.node.get_parameter("velocity_topic").get_parameter_value().string_value
        dead_reckoning_topic = self.node.get_parameter("dead_reckoning_topic").get_parameter_value().string_value
        odometry_topic = self.node.get_parameter("odometry_topic").get_parameter_value().string_value
        
        # Velocity status
        self.velocity_sub = self.node.create_subscription(
            DVL,
            velocity_topic,
            self.velocity_callback,
            self.qos
        )
        # overall Status for a given update loop
        self._vel_status: tuple[bytes, str] = (DiagnosticStatus.STALE, "No velocity data received yet.")
        # additional details provided as key-value pairs in the diagnostics message
        self._vel_status_details:dict = {}
        # Add velocity status check to diagnostics updater
        # use lambda so updater can be updated in real time (kind of like pointer reference in C++)
        self.assign_status_to_updater("DVL Velocity Status", lambda: self._vel_status, lambda: self._vel_status_details)
               
        self.dr_sub = self.node.create_subscription(
            DVLDR,
            dead_reckoning_topic,
            self.dead_reckoning_callback,
            self.qos
        )
        # overall Status for a given update loop
        self._dr_status: tuple[bytes, str] = (DiagnosticStatus.STALE, "No dead reckoning data received yet.")
        # additional details provided as key-value pairs in the diagnostics message
        self._dr_status_details:dict = {}
        # Add dead reckoning status check to diagnostics updater
        # use lambda so updater can be updated in real time (kind of like pointer reference in C++)
        self.assign_status_to_updater("DVL Dead Reckoning Status", lambda: self._dr_status, lambda:self._dr_status_details)
        
        # odometry sub simply updates the last update time for the stale check in diagnostics, no additional status checks implemented
        self.odom_sub = self.node.create_subscription(
            Odometry,
            odometry_topic,
            self.odometry_callback,
            self.qos
        )
        
        self.last_update_time:Time | None = None
        self.log_init()
        self.node.get_logger().warn("DVL Monitor assumes that DVL is mounted such that LED is pointing forward (cable backward).")

    def get_params_for_log(self) -> list[tuple[str, str]]:
        return [
            ("DVL beam count", str(self.dvl_beam_count)),
            ("Velocity topic", self.velocity_sub.topic),
            ("Dead reckoning topic", self.dr_sub.topic),
            ("Odometry topic", self.odom_sub.topic),
        ]
    
    def get_last_update_time(self) -> Time | None:
        # updated in odometry_callback since that is the messag we actually care about
        return self.last_update_time
    
    def velocity_callback(self, msg:DVL) -> None:
        # Reset detail from previous callback — prevents stale entries persisting
        self._vel_status_details = {}

        if not msg.velocity_valid:
            self._vel_status = (DiagnosticStatus.WARN, "DVL velocity invalid")
            return
        
        if msg.status == 1:
            self._vel_status_details["⚠ Thermal warning"] = (
                "<b>DVL may be overheating.</b> Ensure DVL is "
                "underwater during operation or disable acoustics."
            )
            self._vel_status = (DiagnosticStatus.WARN,"⚠ DVL OVERHEATING WARNING — monitor temperature")
        if msg.status > 1:
            self._vel_status_details["Status code"] = f"<b>Unknown: {msg.status}</b>"
            self._vel_status = (DiagnosticStatus.WARN, f"Invalid DVL velocity status code")
            return
        
        if msg.altitude < 0:
            self._vel_status_details["Altitude"] = (
                f"<b>{msg.altitude:.3f} m</b> — expected &ge; 0"
            )
            self._vel_status = (DiagnosticStatus.WARN, "DVL velocity altitude negative")
            return
        
        invalid_beams: list[str] = []
        for beam in msg.beams:
            if not beam.valid:
                invalid_beams.append(DVLMonitor.BEAMIDX_TO_NAME.get(beam.id, f"Unknown({beam.id})"))
                self._vel_status_details[f"{DVLMonitor.BEAMIDX_TO_NAME.get(beam.id, f'Unknown({beam.id})')} Beam"] = "Invalid"
            else:
                self._vel_status_details[f"{DVLMonitor.BEAMIDX_TO_NAME.get(beam.id, f'Unknown({beam.id})')} Beam"] = "Valid"
        valid_beam_count = self.dvl_beam_count - len(invalid_beams)
        if invalid_beams:
            self._vel_status_details["Beam validity"] = (
                f"<b>{valid_beam_count}/{self.dvl_beam_count}</b> beams valid &nbsp;|&nbsp; "
                f"Invalid: <b>{', '.join(invalid_beams)}</b>"
            )
            self._vel_status = (
                DiagnosticStatus.WARN,
                f"DVL has {len(invalid_beams)} invalid beam(s): {', '.join(invalid_beams)}",
            )
            return

        self._vel_status_details["Beam validity"] = (
            f"<b>{self.dvl_beam_count}/{self.dvl_beam_count}</b> beams valid"
        )
        self._vel_status_details["Altitude"] = f"{msg.altitude:.3f} m"
        self._vel_status = (DiagnosticStatus.OK, "DVL velocity healthy")

    def dead_reckoning_callback(self, msg:DVLDR) -> None:
        # Reset detail from previous callback — prevents stale entries persisting
        self._dr_status_details = {}
        
        if msg.status > 0:
            self._dr_status_details["Status code"] = f"<b>Error: {msg.status}</b>"
            self._dr_status = (DiagnosticStatus.WARN, f"Invalid DVL dead reckoning status code")
            return        
        self._dr_status = (DiagnosticStatus.OK, "DVL dead reckoning healthy")

    
    def odometry_callback(self, msg:Odometry) -> None:
        # Update last update time for diagnostics stale check
        self.last_update_time = self.node.get_clock().now()
        
        
def main(args = None):
    rclpy.init(args=args)
    node:Node = rclpy.create_node("dvl_monitor")
    DVLMonitor(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == "__main__":
    main(args=sys.argv)