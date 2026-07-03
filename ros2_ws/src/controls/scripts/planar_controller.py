#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from controls.pid import PID

from geometry_msgs.msg import Wrench, PointStamped
from std_msgs.msg import Float64


class PlanarCoordinator:
    def __init__(self):
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.max_slew_rate_x = 0.0
        self.max_slew_rate_y = 0.0
        self.enabled_x = False
        self.enabled_y = False
        self.last_update_time = None

    def update(self, time_step):
        now = time.time()
        if self.last_update_time is not None and (now - self.last_update_time) < (time_step * 0.5):
            return
        self.last_update_time = now

        enabled = self.enabled_x or self.enabled_y
        rates = [r for r in [self.max_slew_rate_x, self.max_slew_rate_y] if r > 0.0]
        max_slew_rate = min(rates) if rates else 0.0

        if not enabled or max_slew_rate <= 0.0:
            self.setpoint_x = self.target_x
            self.setpoint_y = self.target_y
            return

        dx = self.target_x - self.setpoint_x
        dy = self.target_y - self.setpoint_y
        dist = math.hypot(dx, dy)
        max_step = max_slew_rate * time_step

        if dist > max_step and dist > 1e-6:
            self.setpoint_x += max_step * (dx / dist)
            self.setpoint_y += max_step * (dy / dist)
        else:
            self.setpoint_x = self.target_x
            self.setpoint_y = self.target_y


class AxisController(Node):
    def __init__(self, axis_name, coordinator):
        super().__init__(f'{axis_name}_controller')

        if axis_name not in ['x', 'y']:
            raise ValueError("axis_name must be 'x' or 'y'")
        
        self.axis_name = axis_name
        self.coordinator = coordinator

        # QoS: Always use most recent readings, reliable, volatile durability
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_effort = self.create_publisher(Wrench, f'/controls/{axis_name}_effort', qos_profile_sensor_data)
        self.sub_position = self.create_subscription(PointStamped, f'auv_frame/dvl/position', self.position_callback, qos_profile_sensor_data)
        self.setpoint_sub = self.create_subscription(Float64, f'/controls/{axis_name}_setpoint', self.setpoint_callback, qos)

        self.declare_parameter('control_loop_hz', 10.0)
        self.declare_parameter("KP", 0.0)
        self.declare_parameter("KD", 0.0)
        self.declare_parameter("KI", 0.0)
        self.declare_parameter("I_MAX", 0.0)
        self.declare_parameter("integral_activation_threshold", 0.5)
        self.declare_parameter("max_slew_rate", 0.0)
        self.declare_parameter("derivative_filter_alpha", 0.8)
        self.declare_parameter("enabled", False)

        # PID controller parameters
        self.control_loop_hz = float(self.get_parameter('control_loop_hz').value)
        self.KP = float(self.get_parameter("KP").value)
        self.KD = float(self.get_parameter("KD").value)
        self.KI = float(self.get_parameter("KI").value)
        self.I_MAX = float(self.get_parameter("I_MAX").value)
        self.integral_activation_threshold = float(self.get_parameter("integral_activation_threshold").value)
        self.max_slew_rate = float(self.get_parameter("max_slew_rate").value)
        self.derivative_filter_alpha = float(self.get_parameter("derivative_filter_alpha").value)
        self.enabled = bool(self.get_parameter("enabled").value)

        if self.axis_name == 'x':
            self.coordinator.max_slew_rate_x = self.max_slew_rate
            self.coordinator.enabled_x = self.enabled
        elif self.axis_name == 'y':
            self.coordinator.max_slew_rate_y = self.max_slew_rate
            self.coordinator.enabled_y = self.enabled

        self.parameter_callback_handle = self.add_on_set_parameters_callback(self.parameters_callback)

        self.pid = PID(self.KP, self.KD, self.KI, self.I_MAX, self.integral_activation_threshold, self.derivative_filter_alpha)

        self.setpoint = 0.0  # Setpoint in meters
        self.target_setpoint = 0.0
        self.current_position = 0.0   # Current position in meters
        self.last_position_time = time.time()
        self.time_step = 1.0 / self.control_loop_hz


        # Timer-based control loop (fires every time_step seconds)
        self.timer = self.create_timer(self.time_step, self.control_loop_callback)

    def position_callback(self, msg):
        now = time.time()
        dt = now - self.last_position_time
        pos = msg.point.x if self.axis_name == 'x' else msg.point.y
        if dt >= 0.05:
            self.pid.update_derivative(pos, dt)
            self.last_position_time = now
        self.current_position = pos

    def setpoint_callback(self, msg):
        self.target_setpoint = msg.data
        if self.axis_name == 'x':
            self.coordinator.target_x = msg.data
            if self.max_slew_rate <= 0.0 or not self.enabled:
                self.coordinator.setpoint_x = msg.data
        elif self.axis_name == 'y':
            self.coordinator.target_y = msg.data
            if self.max_slew_rate <= 0.0 or not self.enabled:
                self.coordinator.setpoint_y = msg.data
        if self.max_slew_rate <= 0.0 or not self.enabled:
            self.setpoint = self.target_setpoint

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
                if new_enabled and not self.enabled:
                    self.setpoint = self.current_position
                    self.target_setpoint = self.current_position
                    if self.axis_name == 'x':
                        self.coordinator.setpoint_x = self.current_position
                        self.coordinator.target_x = self.current_position
                    elif self.axis_name == 'y':
                        self.coordinator.setpoint_y = self.current_position
                        self.coordinator.target_y = self.current_position
                self.enabled = new_enabled
                self.get_logger().info(f"{self.axis_name} controller enabled: {self.enabled}")
            elif parameter.name == "max_slew_rate":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'max_slew_rate' must be a double/int"
                    return result
                self.max_slew_rate = float(parameter.value)
                self.get_logger().info(f"Updated {self.axis_name} max_slew_rate: {self.max_slew_rate:.4f}")
            elif parameter.name == "KP":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'KP' must be a double/int"
                    return result
                self.KP = float(parameter.value)
                self.pid.KP = self.KP
                self.get_logger().info(f"Updated {self.axis_name} KP: {self.KP:.4f}")
            elif parameter.name == "KI":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'KI' must be a double/int"
                    return result
                self.KI = float(parameter.value)
                self.pid.KI = self.KI
                self.get_logger().info(f"Updated {self.axis_name} KI: {self.KI:.4f}")
            elif parameter.name == "KD":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'KD' must be a double/int"
                    return result
                self.KD = float(parameter.value)
                self.pid.KD = self.KD
                self.get_logger().info(f"Updated {self.axis_name} KD: {self.KD:.4f}")
            elif parameter.name == "I_MAX":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'I_MAX' must be a double/int"
                    return result
                self.I_MAX = float(parameter.value)
                self.pid.I_MAX = self.I_MAX
                self.get_logger().info(f"Updated {self.axis_name} I_MAX: {self.I_MAX:.4f}")
            elif parameter.name == "integral_activation_threshold":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'integral_activation_threshold' must be a double/int"
                    return result
                self.integral_activation_threshold = float(parameter.value)
                self.pid.integral_activation_threshold = self.integral_activation_threshold
                self.get_logger().info(f"Updated {self.axis_name} integral_activation_threshold: {self.integral_activation_threshold:.4f}")
            elif parameter.name == "derivative_filter_alpha":
                if parameter.type_ not in [parameter.Type.DOUBLE, parameter.Type.INTEGER]:
                    result.successful = False
                    result.reason = "'derivative_filter_alpha' must be a double/int"
                    return result
                self.derivative_filter_alpha = float(parameter.value)
                self.pid.derivative_filter_alpha = self.derivative_filter_alpha
                self.get_logger().info(f"Updated {self.axis_name} derivative_filter_alpha: {self.derivative_filter_alpha:.4f}")

        return result

    def control_loop_callback(self):
        # Sync current node's rate and enabled state to shared coordinator
        if self.axis_name == 'x':
            self.coordinator.max_slew_rate_x = self.max_slew_rate
            self.coordinator.enabled_x = self.enabled
        elif self.axis_name == 'y':
            self.coordinator.max_slew_rate_y = self.max_slew_rate
            self.coordinator.enabled_y = self.enabled
        # Update shared 2D planar slew-rate coordinator
        self.coordinator.update(self.time_step)
        if self.axis_name == 'x':
            self.setpoint = self.coordinator.setpoint_x
        elif self.axis_name == 'y':
            self.setpoint = self.coordinator.setpoint_y

        # Publish effort command
        effort_msg = Wrench()
        if self.enabled:
            self.pid.compute_errors(self.setpoint, self.current_position, self.time_step)
            effort_output = self.pid.compute_effort()
            effort_msg.force.x = effort_output if self.axis_name == 'x' else 0.0
            effort_msg.force.y = effort_output if self.axis_name == 'y' else 0.0
        self.pub_effort.publish(effort_msg)  # Published effort is in pool frame


def main():
    rclpy.init()
    coordinator = PlanarCoordinator()
    x_controller = AxisController('x', coordinator)
    y_controller = AxisController('y', coordinator)
    executor = SingleThreadedExecutor()
    executor.add_node(x_controller)
    executor.add_node(y_controller)
    try:
        executor.spin()
    finally:
        executor.remove_node(x_controller)
        executor.remove_node(y_controller)
        x_controller.destroy_node()
        y_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
