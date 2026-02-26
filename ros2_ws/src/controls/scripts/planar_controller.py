#!/usr/bin/env python3

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from controls.pid import PID

from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64


class AxisController(Node):
    def __init__(self, axis_name):
        super().__init__(f'{axis_name}_controller')

        if axis_name not in ['x', 'y']:
            raise ValueError("axis_name must be 'x' or 'y'")
        
        self.axis_name = axis_name

        # QoS: Always use most recent readings, reliable, volatile durability
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_effort = self.create_publisher(Wrench, f'/controls/{axis_name}_effort', qos)
        self.sub_position = self.create_subscription(Float64, f'auv_frame/{axis_name}', self.position_callback, qos)
        self.setpoint_sub = self.create_subscription(Float64, f'/controls/{axis_name}_setpoint', self.setpoint_callback, qos)

        self.declare_parameter('control_loop_hz', 10.0)
        self.declare_parameter("KP", 0.0)
        self.declare_parameter("KD", 0.0)
        self.declare_parameter("KI", 0.0)
        self.declare_parameter("I_MAX", 0.0)
        self.declare_parameter("enabled", False)

        # PID controller parameters
        self.control_loop_hz = float(self.get_parameter('control_loop_hz').value)
        self.KP = float(self.get_parameter("KP").value)
        self.KD = float(self.get_parameter("KD").value)
        self.KI = float(self.get_parameter("KI").value)
        self.I_MAX = float(self.get_parameter("I_MAX").value)
        self.enabled = bool(self.get_parameter("enabled").value)

        self.parameter_callback_handle = self.add_on_set_parameters_callback(self.parameters_callback)

        self.pid = PID(self.KP, self.KD, self.KI, self.I_MAX)

        self.setpoint = 0.0  # Setpoint in meters
        self.current_position = 0.0   # Current position in meters
        self.previous_position = 0.0  # Previous position for derivative calculation
        self.time_step = 1.0 / self.control_loop_hz


        # Timer-based control loop (fires every time_step seconds)
        self.timer = self.create_timer(self.time_step, self.control_loop_callback)

    def position_callback(self, msg):
        self.previous_position = self.current_position
        self.current_position = msg.data

    def setpoint_callback(self, msg):
        self.setpoint = msg.data

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
                self.get_logger().info(f"{self.axis_name} controller enabled: {self.enabled}")

        return result

    def control_loop_callback(self):
        # Publish effort command
        effort_msg = Wrench()
        if self.enabled:
            self.pid.compute_errors(self.setpoint, self.current_position, self.previous_position, self.time_step)
            effort_output = self.pid.compute_effort()
            effort_msg.force.x = effort_output if self.axis_name == 'x' else 0.0
            effort_msg.force.y = effort_output if self.axis_name == 'y' else 0.0
        self.pub_effort.publish(effort_msg)  # Published effort is in pool frame


def main():
    rclpy.init()
    x_controller = AxisController('x')
    y_controller = AxisController('y')
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
