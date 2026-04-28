from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.time import Time

import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

from abc import ABC, abstractmethod

class HealthMonitor(ABC):
    def __init__(self, node:Node):
        # Diagnostic Updater setup
        self.node = node
        self.updater = diagnostic_updater.Updater(node)
        
        # Parameter declaration
        self.node.declare_parameter("hardware_id", Parameter.Type.STRING)
        self.node.declare_parameter("stale_threshold_sec", Parameter.Type.DOUBLE) # in seconds
        
        # Parameter initialization
        hardware_id:str = self.node.get_parameter("hardware_id").get_parameter_value().string_value
        stale_threshold = self.node.get_parameter("stale_threshold_sec").get_parameter_value().double_value

        self.stale_threshold:Duration = Duration(nanoseconds=int(stale_threshold * 1e9)) # convert seconds to nanoseconds for Duration
        self.updater.setHardwareID(hardware_id)
        self.updater.add(self.check_sensor)
        
        self.last_update_time:Time = self.node.get_clock().now()
        

    def check_sensor(self, stat:diagnostic_updater.DiagnosticStatusWrapper):
        """
        Diagnostic callback to check sensor health. Checks if the sensor data is stale based on the last update time and the defined threshold, then checks additional sensor-specific status conditions implemented in the check_status method.
        """
        
        time_since_last_update:Duration = self.node.get_clock().now() - self.last_update_time
        
        # Check if data is stale
        if time_since_last_update > self.stale_threshold:
            stat.summary(DiagnosticStatus.STALE, f"No data received within the stale threshold of {self.stale_threshold.nanoseconds / 1e9:.2f} seconds.")
        else:
            # If not stale, check additional sensor-specific conditions
            if self.check_status(stat):
                stat.summary(DiagnosticStatus.OK, "Running.")
            else:
                stat.summary(DiagnosticStatus.ERROR, "Sensor status check failed.")
        return stat

    @abstractmethod
    def check_status(self, stat:diagnostic_updater.DiagnosticStatusWrapper) -> bool:
        pass