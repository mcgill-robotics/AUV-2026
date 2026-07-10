#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
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

        self.pub_effort = self.create_publisher(Wrench, '/controls/depth_effort', qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(Float64, 'auv_frame/depth', self.depth_callback, qos_profile_sensor_data)
        self.setpoint_sub = self.create_subscription(Float64, '/controls/depth_setpoint', self.setpoint_callback, qos)

        self.declare_parameter('control_loop_hz', 10.0)
        self.declare_parameter("KP", 0.0)
        self.declare_parameter("KD", 0.0)
        self.declare_parameter("KI", 0.0)
        self.declare_parameter("I_MAX", 0.0)
        self.declare_parameter("integral_activation_threshold", 1.0)
        self.declare_parameter("net_buoyancy", 0.0)
        self.declare_parameter("max_slew_rate", 0.0)
        self.declare_parameter("derivative_filter_alpha", 0.2)
        self.declare_parameter("enabled", False)

        # PID controller parameters
        self.control_loop_hz = float(self.get_parameter('control_loop_hz').value)
        self.KP = float(self.get_parameter("KP").value)
        self.KD = float(self.get_parameter("KD").value)
        self.KI = float(self.get_parameter("KI").value)
        self.I_MAX = float(self.get_parameter("I_MAX").value)
        self.integral_activation_threshold = float(self.get_parameter("integral_activation_threshold").value)
        self.net_buoyancy = float(self.get_parameter("net_buoyancy").value)
        self.max_slew_rate = float(self.get_parameter("max_slew_rate").value)
        self.derivative_filter_alpha = float(self.get_parameter("derivative_filter_alpha").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.was_enabled = self.enabled

        self.parameter_callback_handle = self.add_on_set_parameters_callback(self.parameters_callback)

        self.pid = PID(self.KP, self.KD, self.KI, self.I_MAX, self.integral_activation_threshold, self.derivative_filter_alpha)

        self.setpoint_depth = 0.25  # Desired depth in meters. TODO: Change default value to AUV float depth
        self.target_setpoint_depth = 0.25
        self.current_depth = 0.0   # Current depth in meters
        self.last_depth_time = time.time()
        self.time_step = 1.0 / self.control_loop_hz

        #Feed-Forward term
        self.feed_forward = -self.net_buoyancy

        # Timer-based control loop (fires every time_step seconds)
        self.timer = self.create_timer(self.time_step, self.control_loop_callback)

    def depth_callback(self, msg):
        now = time.time()
        dt = now - self.last_depth_time
        if dt >= 0.05:
            # +Z is up, so invert depth values for PID calculations
            self.pid.update_derivative(-msg.data, dt)
            self.last_depth_time = now
        self.current_depth = msg.data

    def setpoint_callback(self, msg):
        if abs(msg.data - self.target_setpoint_depth) > 1e-3:
            self.pid.integral_error = 0.0
        self.target_setpoint_depth = msg.data
        if self.max_slew_rate <= 0.0 or not self.enabled:
            self.setpoint_depth = self.target_setpoint_depth

    def parameters_callback(self, parameters):
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            if parameter.name == "enabled":
                if parameter.type_ != parameter.Type.BOOL:
                    result.successful = False
                    result.reason = "'enabled' must be a bool"
                    return result
                new_enabled = bool(parameter.value)
                if new_enabled != self.enabled:
                    self.pid.integral_error = 0.0
                if new_enabled and not self.enabled:
                    self.setpoint_depth = self.current_depth
                    self.target_setpoint_depth = self.current_depth
                self.enabled = new_enabled
                self.get_logger().info(f"Depth controller enabled: {self.enabled}")
            elif parameter.name == "max_slew_rate":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'max_slew_rate' must be a double/int"
                    return result
                self.max_slew_rate = float(parameter.value)
                self.get_logger().info(f"Updated max_slew_rate: {self.max_slew_rate:.4f}")
            elif parameter.name == "KP":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'KP' must be a double/int"
                    return result
                self.KP = float(parameter.value)
                self.pid.KP = self.KP
                self.get_logger().info(f"Updated KP: {self.KP:.4f}")
            elif parameter.name == "KI":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'KI' must be a double/int"
                    return result
                self.KI = float(parameter.value)
                self.pid.KI = self.KI
                self.get_logger().info(f"Updated KI: {self.KI:.4f}")
            elif parameter.name == "KD":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'KD' must be a double/int"
                    return result
                self.KD = float(parameter.value)
                self.pid.KD = self.KD
                self.get_logger().info(f"Updated KD: {self.KD:.4f}")
            elif parameter.name == "I_MAX":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'I_MAX' must be a double/int"
                    return result
                self.I_MAX = float(parameter.value)
                self.pid.I_MAX = self.I_MAX
                self.get_logger().info(f"Updated I_MAX: {self.I_MAX:.4f}")
            elif parameter.name == "integral_activation_threshold":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'integral_activation_threshold' must be a double/int"
                    return result
                self.integral_activation_threshold = float(parameter.value)
                self.pid.integral_activation_threshold = self.integral_activation_threshold
                self.get_logger().info(f"Updated integral_activation_threshold: {self.integral_activation_threshold:.4f}")
            elif parameter.name == "derivative_filter_alpha":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'derivative_filter_alpha' must be a double/int"
                    return result
                self.derivative_filter_alpha = float(parameter.value)
                self.pid.derivative_filter_alpha = self.derivative_filter_alpha
                self.get_logger().info(f"Updated derivative_filter_alpha: {self.derivative_filter_alpha:.4f}")
            elif parameter.name == "net_buoyancy":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'net_buoyancy' must be a double/int"
                    return result
                self.net_buoyancy = float(parameter.value)
                self.feed_forward = -self.net_buoyancy
                self.get_logger().info(f"Updated net_buoyancy: {self.net_buoyancy:.4f}, feed_forward: {self.feed_forward:.4f}")

        return result

    def control_loop_callback(self):
        # Apply setpoint slew-rate limiting if enabled
        is_slewing = False
        if self.enabled and self.max_slew_rate > 0.0:
            max_step = self.max_slew_rate * self.time_step
            diff = self.target_setpoint_depth - self.setpoint_depth
            if abs(diff) > max_step:
                self.setpoint_depth += math.copysign(max_step, diff)
                is_slewing = True
            else:
                self.setpoint_depth = self.target_setpoint_depth
        else:
            self.setpoint_depth = self.target_setpoint_depth

        # Publish effort command
        if self.enabled:
            effort_msg = Wrench()
            # +Z is up, so invert depth values for PID calculations
            self.pid.compute_errors(-self.setpoint_depth, -self.current_depth, self.time_step, allow_integration=not is_slewing)
            effort_output = self.pid.compute_effort()
            effort_msg.force.z = effort_output + self.feed_forward
            self.pub_effort.publish(effort_msg)  # Published effort is in pool frame
            self.was_enabled = True
        elif self.was_enabled:
            effort_msg = Wrench()
            effort_msg.force.z = 0.0
            self.pub_effort.publish(effort_msg)
            self.was_enabled = False


def main():
    rclpy.init()
    depth_controller = DepthController()
    rclpy.spin(depth_controller)
    depth_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
