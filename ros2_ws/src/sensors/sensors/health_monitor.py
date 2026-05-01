from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.time import Time

from diagnostic_updater import Updater, DiagnosticStatusWrapper
from diagnostic_msgs.msg import DiagnosticStatus

from abc import ABC, abstractmethod

"""
A base class for monitoring the health of sensors in a ROS2 system. This class uses the diagnostic_updater package to create a diagnostic updater that checks the health of a sensor based on the time since the last update and additional sensor-specific conditions defined in derived classes. The class declares parameters for hardware ID and stale data threshold, expects status checks to be added to its Updater.
"""
class HealthMonitor(ABC):
    # copy QOS setting from qos_profile_sensor_data since we need to set depth to 1 (default 5)
    # and there is no way to directly modify the default qos_profile_sensor_data
    qos: QoSProfile = QoSProfile(
        reliability=qos_profile_sensor_data.reliability,
        durability=qos_profile_sensor_data.durability,
        history=qos_profile_sensor_data.history,
        depth=1
    )
    def __init__(self, node:Node):
        # Diagnostic Updater setup
        self.node = node
        self.updater = Updater(node)
        
        # Parameter declaration
        self.node.declare_parameter("hardware_id", Parameter.Type.STRING)
        self.node.declare_parameter("stale_threshold_sec", Parameter.Type.DOUBLE) # in seconds
        
        # Parameter initialization
        hardware_id:str = self.node.get_parameter("hardware_id").get_parameter_value().string_value
        stale_threshold:float = self.node.get_parameter("stale_threshold_sec").get_parameter_value().double_value
        
        self.stale_threshold:Duration = Duration(nanoseconds=int(stale_threshold * 1e9)) # convert seconds to nanoseconds for Duration
        self.last_update_time:Time | None = None
        
        self.updater.setHardwareID(hardware_id)
        self.updater.add("Sensor Health Check", self.check_sensor_stale)
        
        

    def check_sensor_stale(self, stat:DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        """
        Diagnostic callback to check sensor health. Checks if the sensor data is stale based on the last update time and the defined threshold
        """
        
        if self.last_update_time is None:
            # If we have never received an update, consider it stale
            stat.summary(DiagnosticStatus.STALE, "No data received yet.")
            return stat
        time_since_last_update:Duration = self.node.get_clock().now() - self.last_update_time
        # Check if data is stale
        elapsed_sec = time_since_last_update.nanoseconds / 1e9
        # Add message
        stat.add(
            "Time since last message",
            f"<b>{elapsed_sec:.2f} s</b> (threshold: {self.stale_threshold.nanoseconds / 1e9:.2f} s)",
        )
        if time_since_last_update > self.stale_threshold:
            # If data is stale, summarize report as STALE and return immediately
            stat.summary(DiagnosticStatus.STALE, "Data stale, no message received within threshold.")
            return stat
        stat.summary(DiagnosticStatus.OK, "Data received")        
        return stat