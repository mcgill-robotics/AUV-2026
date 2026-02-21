#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from controls.pid import PID

from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64


class DepthController(Node):
    def __init__(self):
        super().__init__('depth_controller')

        # QoS: Always use most recent readings, reliable, volatile durability
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_effort = self.create_publisher(Wrench, '/controls/depth_effort', qos)
        self.sub_depth = self.create_subscription(Float64, 'auv_frame/depth', self.depth_callback, qos)
        self.setpoint_sub = self.create_subscription(Float64, '/controls/depth_setpoint', self.setpoint_callback, qos)

        self.declare_parameter('control_loop_hz', 10.0)
        self.declare_parameter("KP", 0.0)
        self.declare_parameter("KD", 0.0)
        self.declare_parameter("KI", 0.0)
        self.declare_parameter("I_MAX", 0.0)
        self.declare_parameter("net_buoyancy", 0.0)
        self.declare_parameter("enabled", False)

        # PID controller parameters
        self.control_loop_hz = float(self.get_parameter('control_loop_hz').value)
        self.KP = float(self.get_parameter("KP").value)
        self.KD = float(self.get_parameter("KD").value)
        self.KI = float(self.get_parameter("KI").value)
        self.I_MAX = float(self.get_parameter("I_MAX").value)
        self.net_buoyancy = float(self.get_parameter("net_buoyancy").value)
        self.enabled = bool(self.get_parameter("enabled").value)

        self.parameter_callback_handle = self.add_on_set_parameters_callback(self.parameters_callback)

        self.pid = PID(self.KP, self.KD, self.KI, self.I_MAX)

        self.setpoint_depth = 0.25  # Desired depth in meters. TODO: Change default value to AUV float depth
        self.current_depth = 0.0   # Current depth in meters
        self.previous_depth = 0.0  # Previous depth for derivative calculation
        self.time_step = 1.0 / self.control_loop_hz

        #Feed-Forward term
        self.feed_forward = -self.net_buoyancy

        # Timer-based control loop (fires every time_step seconds)
        self.timer = self.create_timer(self.time_step, self.control_loop_callback)

    def depth_callback(self, msg):
        self.previous_depth = self.current_depth
        self.current_depth = msg.data

    def setpoint_callback(self, msg):
        self.setpoint_depth = msg.data

    def parameters_callback(self, parameters):
        result = SetParametersResult()
        result.successful = True

        for parameter in parameters:
            if parameter.name == "enabled":
                if parameter.type_ != parameter.Type.BOOL:
                    result.successful = False
                    result.reason = "'enabled' must be a bool"
                    return result
                self.enabled = bool(parameter.value)
                self.get_logger().info(f"Depth controller enabled: {self.enabled}")

        return result

    def control_loop_callback(self):
        # Publish effort command
        effort_msg = Wrench()
        if self.enabled:
            # +Z is up, so invert depth values for PID calculations
            self.pid.compute_errors(-self.setpoint_depth, -self.current_depth, -self.previous_depth, self.time_step)
            effort_output = self.pid.compute_output()
            effort_msg.force.z = effort_output + self.feed_forward
        else:
            effort_msg.force.z = 0.0

        self.pub_effort.publish(effort_msg)  # Published effort is in pool frame


def main():
    rclpy.init()
    depth_controller = DepthController()
    rclpy.spin(depth_controller)
    depth_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
