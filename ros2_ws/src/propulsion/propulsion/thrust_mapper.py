#!/usr/bin/env python3
"""
Description: Thrust mapper node subscribes to the effort topic, converts the wrench readings to thruster forces,
and then converts the forces to PWM signals and publishes them.

Subscribes:  /controls/total_effort (geometry_msgs/msg/Wrench)
Publishes:   /propulsion/microseconds (std_msgs/msg/Int16MultiArray)
             /propulsion/forces      (auv_msgs/msg/ThrusterForces)
Parameters:  a, b, c, d, e, dx, dy (m), alpha (deg), thruster force and PWM limits. 
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from propulsion.force_to_pwm_converter import force_to_pwm_thruster
from propulsion.thrust_allocator import ThrustAllocator
from auv_msgs.msg import ThrusterForces
from geometry_msgs.msg import Wrench
from std_msgs.msg import Int16MultiArray


class ThrusterMapper(Node):
    def __init__(self):
        super().__init__('thrust_mapper')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_us = self.create_publisher(Int16MultiArray, 'propulsion/microseconds', qos_reliable)
        self.pub_forces = self.create_publisher(ThrusterForces, 'propulsion/forces', qos_reliable)

        self.declare_parameter('max_thrust_t1', 40.0)
        self.declare_parameter('max_thrust_t2', 40.0)
        self.declare_parameter('max_thrust_t3', 40.0)
        self.declare_parameter('max_thrust_t4', 40.0)
        self.declare_parameter('max_thrust_t5', 40.0)
        self.declare_parameter('max_thrust_t6', 40.0)
        self.declare_parameter('max_thrust_t7', 40.0)
        self.declare_parameter('max_thrust_t8', 40.0)

        self.declare_parameter('min_thrust_t1', -40.0)
        self.declare_parameter('min_thrust_t2', -40.0)
        self.declare_parameter('min_thrust_t3', -40.0)
        self.declare_parameter('min_thrust_t4', -40.0)
        self.declare_parameter('min_thrust_t5', -40.0)
        self.declare_parameter('min_thrust_t6', -40.0)
        self.declare_parameter('min_thrust_t7', -40.0)
        self.declare_parameter('min_thrust_t8', -40.0)

        self.declare_parameter('thruster_PWM_lower_limit', 1100)
        self.declare_parameter('thruster_PWM_upper_limit', 1900)
        self.declare_parameter('watchdog_timeout_sec', 1.0)  # Reset thrusters if no wrench received for this long
        self.declare_parameter('arming_delay_sec', 1.0)  # Seconds to wait before accepting wrench commands


        # Geometric parameters of the thruster positions
        # Consult README for reference axes and dimensions
        # Units are in Degrees and m
        self.declare_parameter('a', float("nan"))
        self.declare_parameter('b', float("nan"))
        self.declare_parameter('c', float("nan"))
        self.declare_parameter('d', float("nan"))
        self.declare_parameter('e', float("nan"))
        self.declare_parameter('dx', float("nan"))
        self.declare_parameter('dy', float("nan"))
        self.declare_parameter('alpha', float("nan"))

        try:
            self.thruster_lower_limit = self.get_parameter('thruster_PWM_lower_limit').value
            self.thruster_upper_limit = self.get_parameter('thruster_PWM_upper_limit').value

            max_thrust_t1 = self.get_parameter('max_thrust_t1').value
            max_thrust_t2 = self.get_parameter('max_thrust_t2').value
            max_thrust_t3 = self.get_parameter('max_thrust_t3').value
            max_thrust_t4 = self.get_parameter('max_thrust_t4').value
            max_thrust_t5 = self.get_parameter('max_thrust_t5').value
            max_thrust_t6 = self.get_parameter('max_thrust_t6').value
            max_thrust_t7 = self.get_parameter('max_thrust_t7').value
            max_thrust_t8 = self.get_parameter('max_thrust_t8').value

            min_thrust_t1 = self.get_parameter('min_thrust_t1').value
            min_thrust_t2 = self.get_parameter('min_thrust_t2').value
            min_thrust_t3 = self.get_parameter('min_thrust_t3').value
            min_thrust_t4 = self.get_parameter('min_thrust_t4').value
            min_thrust_t5 = self.get_parameter('min_thrust_t5').value
            min_thrust_t6 = self.get_parameter('min_thrust_t6').value
            min_thrust_t7 = self.get_parameter('min_thrust_t7').value
            min_thrust_t8 = self.get_parameter('min_thrust_t8').value

            a = self._get_float('a')
            b = self._get_float('b')
            c = self._get_float('c')
            d = self._get_float('d')
            e = self._get_float('e')
            dx = self._get_float('dx')
            dy = self._get_float('dy')
            alpha_deg = self._get_float('alpha')
        except Exception as ex:
            self.get_logger().fatal(f'Missing or invalid parameters: {ex}')
            raise

        self.thruster_max_force = [
            max_thrust_t1, max_thrust_t2, max_thrust_t3, max_thrust_t4,
            max_thrust_t5, max_thrust_t6, max_thrust_t7, max_thrust_t8
        ]
        self.thruster_min_force = [
            min_thrust_t1, min_thrust_t2, min_thrust_t3, min_thrust_t4,
            min_thrust_t5, min_thrust_t6, min_thrust_t7, min_thrust_t8
        ]

        alpha = math.radians(alpha_deg)

        self.T = np.array([
            # SURGE (X)
            [ np.cos(alpha), 0, 0, -np.cos(alpha), -np.cos(alpha), 0, 0, np.cos(alpha)],
            # SWAY (Y)
            [ -np.sin(alpha), 0, 0, -np.sin(alpha), np.sin(alpha), 0, 0, np.sin(alpha)],
            # HEAVE (Z)
            [ 0, -1, -1, 0, 0, -1, -1, 0],
            # ROLL (X-TORQUE)
            [ np.sin(alpha)*e, (b+dy), (b+dy), np.sin(alpha)*e, -np.sin(alpha)*e, -(b-dy), -(b-dy), -np.sin(alpha)*e],
            # PITCH (Y-TORQUE)
            [ np.cos(alpha)*e, -(a+dx), (a-dx), -np.cos(alpha)*e, -np.cos(alpha)*e, (a-dx), -(a+dx), np.cos(alpha)*e],
            # YAW (Z-TORQUE)
            [ (np.cos(alpha)*(c+dy) + np.sin(alpha)*(d+dx)), 0, 0, -(np.cos(alpha)*(c+dy) + np.sin(alpha)*(d-dx)), (np.cos(alpha)*(c-dy) + np.sin(alpha)*(d-dx)), 0, 0, -(np.cos(alpha)*(c-dy) + np.sin(alpha)*(d+dx))]
        ])

        self.thrust_allocator = ThrustAllocator(
            self.T,
            np.array(self.thruster_max_force, dtype=float),
            np.array(self.thruster_min_force, dtype=float)
        )

        # QoS profile stored for deferred subscription creation
        self._wrench_qos = qos

        # Watchdog: reset thrusters if no wrench msg received within timeout
        self._watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value
        self._last_wrench_time = None
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        # Arming: send neutral PWM first, then create the wrench subscription
        # after the arming delay. No wrench messages can interfere during arming
        # because the subscription does not exist yet.
        arming_delay = self.get_parameter('arming_delay_sec').value
        self.sub_cmd = None
        self.re_arm()
        self._arming_timer = self.create_timer(
            arming_delay, self._activate_after_arming
        )

    def _get_float(self, name: str) -> float:
        p: Parameter = self.get_parameter(name)
        v = p.value

        if isinstance(v, (float, int)):
            v = float(v)
            if math.isnan(v):
                raise ValueError(f'Parameter {name} is NaN (unset)')
            return v
        raise ValueError(f'Parameter {name} is not a number (got {type(v)})')

    def _activate_after_arming(self):
        """Called once after the arming delay. Creates the wrench subscription
        so that thruster commands can now flow through."""
        self._arming_timer.cancel()
        self.sub_cmd = self.create_subscription(
            Wrench,
            'controls/total_effort',
            self.wrench_to_thrust,
            self._wrench_qos
        )
        self.get_logger().info('Arming complete - wrench subscription active.')

    def _watchdog_callback(self):
        """Resets thrusters to neutral if no wrench message has been received
        within the watchdog timeout. Prevents the AUV from drifting on stale
        Teensy PWM values."""
        if self._last_wrench_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_wrench_time).nanoseconds / 1e9
        if elapsed > self._watchdog_timeout:
            self.get_logger().warn(
                f'Watchdog: no wrench received for {elapsed:.2f}s - resetting thrusters.'
            )
            self.shutdown_thrusters()
            self._last_wrench_time = None

    def wrench_to_thrust(self, wrench_msg: Wrench):
        self._last_wrench_time = self.get_clock().now()

        wrench_vec = np.array([
            wrench_msg.force.x,
            wrench_msg.force.y,
            wrench_msg.force.z,
            wrench_msg.torque.x,
            wrench_msg.torque.y,
            wrench_msg.torque.z
        ], dtype=float)

        try:
            thrust_forces, alpha, method = self.thrust_allocator.allocate_thrust(wrench_vec)
        except RuntimeError as ex:
            self.get_logger().warn(f"Thrust allocator failed: {ex}")
            self.shutdown_thrusters()
            return
        if alpha < 0.99:
            self.get_logger().warn(f"Thrust allocation alpha is below 1: {alpha:.3f} (method: {method})")

        tf_msg = ThrusterForces()
        tf_msg.back_right = float(thrust_forces[0])
        tf_msg.heave_back_right = float(thrust_forces[1])
        tf_msg.heave_front_right = float(thrust_forces[2])
        tf_msg.front_right = float(thrust_forces[3])
        tf_msg.front_left = float(thrust_forces[4])
        tf_msg.heave_front_left = float(thrust_forces[5])
        tf_msg.heave_back_left = float(thrust_forces[6])
        tf_msg.back_left = float(thrust_forces[7])

        self.pub_forces.publish(tf_msg)
        self.forces_to_pwm_publisher(thrust_forces)

    def forces_to_pwm_publisher(self, thrust_forces: np.ndarray):
        pwm_arr = [force_to_pwm_thruster(i + 1, float(thrust_forces[i])) for i in range(8)]
        pwm_arr = np.clip(pwm_arr, self.thruster_lower_limit, self.thruster_upper_limit)
        pwm_arr = pwm_arr.astype(np.uint16, copy=False)

        pwm_msg = Int16MultiArray()
        pwm_msg.data = pwm_arr.tolist()
        self.pub_us.publish(pwm_msg)

    def re_arm(self):
        """Send neutral PWM to initialize thrusters."""
        msg = Int16MultiArray(data=[1500] * 8)
        self.pub_us.publish(msg)
        self.get_logger().info('Sent arming signal (1500µs).')

    def shutdown_thrusters(self):
        msg = Int16MultiArray(data=[1500] * 8)
        self.pub_us.publish(msg)


def main():
    rclpy.init()
    thrust_mapper = ThrusterMapper()
    try:
        rclpy.spin(thrust_mapper)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            thrust_mapper.shutdown_thrusters()
        except Exception:
            pass
        thrust_mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
