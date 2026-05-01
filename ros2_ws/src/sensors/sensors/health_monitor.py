from os import stat

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.time import Time

from diagnostic_updater import Updater, DiagnosticStatusWrapper
from diagnostic_msgs.msg import DiagnosticStatus

from abc import ABC, abstractmethod

"""
A base class for monitoring the health of sensors in a ROS2 system. This class uses the diagnostic_updater package to create a diagnostic updater that checks the health of a sensor based on the time since the last update and additional sensor-specific conditions defined in derived classes. The class declares parameters for hardware ID and stale data threshold, and provides an abstract method for checking sensor-specific status conditions.
"""
class HealthMonitor(ABC):
    qos = QoSProfile (
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
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
        self.updater.add("Health Check", self.check_sensor)
        
        

    def check_sensor(self, stat:DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        """
        Diagnostic callback to check sensor health. Checks if the sensor data is stale based on the last update time and the defined threshold, then checks additional sensor-specific status conditions implemented in the check_status method.
        """
        
        if self.last_update_time is None:
            # If we have never received an update, consider it stale
            stat.summary(DiagnosticStatus.STALE, "No data received yet.")
            return stat
        time_since_last_update:Duration = self.node.get_clock().now() - self.last_update_time
        # Check if data is stale
        if time_since_last_update > self.stale_threshold:
            elapsed_sec = time_since_last_update.nanoseconds / 1e9
            # Add message
            stat.add(
                "Time since last message",
                f"<b>{elapsed_sec:.2f} s</b> (threshold: {self.stale_threshold.nanoseconds / 1e9:.2f} s)",
            )

            # If data is stale, summarize report as STALE and return immediately
            stat.summary(DiagnosticStatus.STALE, "Data stale, no message received within threshold.")
            return stat
        # If not stale, check additional sensor-specific conditions
        return self.check_status(stat)
        

    @abstractmethod
    def check_status(self, stat:DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        """
        Abstract method to be implemented by derived classes to check specific sensor status conditions and update the diagnostic status accordingly.
        Expects both message and summaries to be set within this method based on the sensor's health conditions.
        """
        pass