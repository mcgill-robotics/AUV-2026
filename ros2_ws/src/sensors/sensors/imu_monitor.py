#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.time import Time

from sensor_msgs.msg import Imu
from xsens_mti_ros2_driver.msg import XsStatusWord
from sensors.health_monitor import HealthMonitor

class IMUMonitor(HealthMonitor):
    def __init__(self, node):
        super().__init__(node)
        # Declare IMU-specific parameters
        self.node.declare_parameter("imu_topic", Parameter.Type.STRING)
        self.node.declare_parameter("imu_status_topic", Parameter.Type.STRING)
        # Assign parameters to instance variables
        self.imu_topic = self.node.get_parameter("imu_topic").get_parameter_value().string_value
        self.imu_status_topic = self.node.get_parameter("imu_status_topic").get_parameter_value().string_value

        # Velocity status
        self.imu_status_sub = self.node.create_subscription(
            XsStatusWord,
            self.imu_status_topic,
            self.imu_status_callback,
            self.qos
        )
        # overall Status for a given update loop
        self._imu_status: tuple[bytes, str] = (DiagnosticStatus.STALE, "No velocity data received yet.")
        # additional details provided as key-value pairs in the diagnostics message
        self._imu_status_details:dict = {}
        # Add velocity status check to diagnostics updater
        # use lambda so updater can be updated in real time (kind of like pointer reference in C++)
        self.assign_status_to_updater("IMU Status", lambda: self._imu_status, lambda: self._imu_status_details)
               
        # imu sub simply updates the last update time for the stale check in diagnostics, no additional status checks implemented
        self.imu_sub = self.node.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            self.qos
        )
        
        self.last_update_time:Time | None = None
        self.log_init()

    def get_params_for_log(self) -> list[tuple[str, str]]:
        return [
            ("IMU Topic", str(self.imu_topic)),
            ("IMU Status Topic", str(self.imu_status_topic)),
        ]
    
    def get_last_update_time(self) -> Time | None:
        # updated in odometry_callback since that is the messag we actually care about
        return self.last_update_time
    
    def imu_status_callback(self, msg:XsStatusWord) -> None:
        # Reset detail from previous callback — prevents stale entries persisting
        self._imu_status_details = {}
        # Check if error bit is set
        if not msg.filter_valid:
            self._imu_status = (DiagnosticStatus.ERROR, "IMU filter invalid")
            return
        if msg.clipping_indication:
            oor_accel = []
            if msg.clipflag_acc_x:
                oor_accel.append("X")
            if msg.clipflag_acc_y:
                oor_accel.append("Y")
            if msg.clipflag_acc_z:
                oor_accel.append("Z")
            self._imu_status_details["Out of Range Acceleration Axes"] = ", ".join(oor_accel) if oor_accel else "None"
            
            oor_vel = []
            if msg.clipflag_gyr_x:
                oor_vel.append("X")
            if msg.clipflag_gyr_y:
                oor_vel.append("Y")
            if msg.clipflag_gyr_z:
                oor_vel.append("Z")
            self._imu_status_details["Out of Range Velocity Axes"] = ", ".join(oor_vel) if oor_vel else "None"
            self._imu_status = (DiagnosticStatus.WARN, "IMU clipping detected")
            return
            
        # If we reach here, the IMU is functioning correctly
        self._imu_status = (DiagnosticStatus.OK, "IMU functioning correctly")

    def imu_callback(self, msg:Imu) -> None:
        # Update last update time for diagnostics stale check
        self.last_update_time = self.node.get_clock().now()
        
        
def main(args = None):
    rclpy.init(args=args)
    node:Node = rclpy.create_node("imu_monitor")
    IMUMonitor(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == "__main__":
    main(args=sys.argv)