#!/usr/bin/env python3
"""
Description: Thrust mapper node subscribes to the effort topic, converts the wrench readings to thruster forces,
and then converts the forces to PWM signals and publishes them.

Subscribes:  /controls/effort (geometry_msgs/msg/Wrench)
Publishes:   /propulsion/microseconds (auv_msgs/msg/ThrusterMicroseconds)
             /propulsion/forces      (auv_msgs/msg/ThrusterForces)
Parameters:  a, b, c, d, e, dx, dy (m), alpha (deg), thruster_PWM_lower_limit, thruster_PWM_upper_limit
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from propulsion.thrust_mapper_utils import force_to_pwm_thruster
from auv_msgs.msg import ThrusterForces
from geometry_msgs.msg import Wrench
from std_msgs.msg import Int16MultiArray


class ThrusterMapper(Node):
    def __init__(self):
        super().__init__('thrust_mapper')

        # QoS: Always use most recent controller signals, reliable, volatile durability
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Publishers
        self.pub_us = self.create_publisher(Int16MultiArray, 'propulsion/microseconds', qos)
        self.pub_forces = self.create_publisher(ThrusterForces, 'propulsion/forces', qos)

        # --- Thruster PWM Limits
        self.declare_parameter('thruster_PWM_lower_limit', 1228)
        self.declare_parameter('thruster_PWM_upper_limit', 1768)

        # Geometric parameters of the thruster positions
        # Consult README for reference axes and dimensions used to declare variables and create allocation matrix below
        # Units are in Degrees and m
        self.declare_parameter('a', float("nan")) # [m]
        self.declare_parameter('b', float("nan")) # [m]
        self.declare_parameter('c', float("nan")) # [m]
        self.declare_parameter('d', float("nan")) # [m]
        self.declare_parameter('e', float("nan")) # [m]
        self.declare_parameter('dx', float("nan")) # [m]
        self.declare_parameter('dy', float("nan")) # [m]
        self.declare_parameter('alpha', float("nan")) # degrees

        # Read parameters (raises if missing and no default)
        try:
            self.thruster_lower_limit = self.get_parameter('thruster_PWM_lower_limit').value
            self.thruster_upper_limit = self.get_parameter('thruster_PWM_upper_limit').value

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

        alpha = math.radians(alpha_deg)

        #Matrix mapping from thruster forces to wrench (6x8) 
        T = np.array([
            # SURGE (X)
            [ np.cos(alpha), 0, 0, -np.cos(alpha), -np.cos(alpha), 0, 0, np.cos(alpha)],
            # SWAY (Y)
            [ -np.sin(alpha), 0, 0, -np.sin(alpha), np.sin(alpha), 0, 0, np.sin(alpha)],
            # HEAVE (Z)
            [ 0, -1, -1, 0, 0, -1, -1, 0],
            # ROLL (X-rotation)
            [ np.sin(alpha)*e, (b+dy), (b+dy), np.sin(alpha)*e, -np.sin(alpha)*e, -(b-dy), -(b-dy), -np.sin(alpha)*e],
            # PITCH (Y-rotation)
            [ np.cos(alpha)*e, -(a+dx), (a-dx), -np.cos(alpha)*e, -np.cos(alpha)*e, (a-dx), -(a+dx), np.cos(alpha)*e],
            # YAW (Z-rotation)
            [ (np.cos(alpha)*(c+dy) + np.sin(alpha)*(d+dx)), 0, 0, -(np.cos(alpha)*(c+dy) + np.sin(alpha)*(d-dx)), (np.cos(alpha)*(c-dy) + np.sin(alpha)*(d-dx)), 0, 0, -(np.cos(alpha)*(c-dy) + np.sin(alpha)*(d+dx))]
        ])
        self.T_inv = np.linalg.pinv(T)  # Pseudo-inverse (8x6)

        # --- Subscriber
        self.sub_cmd = self.create_subscription(
            Wrench,
            'controls/effort',
            self.wrench_to_thrust,
            qos
        )

        # Small startup delay for hardware sync via a one-shot timer
        self._arming_timer = self.create_timer(1.0, self._do_arming_once)
        self._arming_done = False


    def _get_float(self, name: str) -> float:
        p: Parameter = self.get_parameter(name)
        v = p.value
  
        if isinstance(v, (float,int)):
            v = float(v)
            if math.isnan(v):
                raise ValueError(f'Parameter {name} is NaN (unset)')
            return v
        else:
            raise ValueError(f'Parameter {name} is not a number (got {type(v)})')


    def _do_arming_once(self):
        if self._arming_done:
            return
        self._arming_done = True
        self._arming_timer.cancel()
        self.re_arm()


    def wrench_to_thrust(self, wrench_msg: Wrench):
        """
        Callback function that maps a received Wrench message into thruster forces by applying the
        pseudo-inverse of the thruster mapping matrix. We assume that all messages published on /controls/effort
        are in the "auv" frame.

        """
        # body wrench vector (6,)
        wrench_vec = np.array([
            wrench_msg.force.x,
            wrench_msg.force.y,
            wrench_msg.force.z,
            wrench_msg.torque.x,
            wrench_msg.torque.y,
            wrench_msg.torque.z
        ], dtype=float)

        # Calculate the thruster forces using the pseudo-inverse
        thrust_forces = self.T_inv @ wrench_vec  # (8,)

        # Publish forces (debug/sim)
        tf_msg = ThrusterForces()
        tf_msg.back_right         = float(thrust_forces[0])
        tf_msg.heave_back_right    = float(thrust_forces[1])
        tf_msg.heave_front_right   = float(thrust_forces[2])
        tf_msg.front_right       = float(thrust_forces[3])
        tf_msg.front_left         = float(thrust_forces[4])
        tf_msg.heave_front_left    = float(thrust_forces[5])
        tf_msg.heave_back_left    = float(thrust_forces[6])
        tf_msg.back_left         = float(thrust_forces[7])

        # Publish the computed thruster forces (useful for simulation/debugging)
        self.pub_forces.publish(tf_msg)

        # Convert to PWM and publish
        self.forces_to_pwm_publisher(thrust_forces)

    def forces_to_pwm_publisher(self, thrust_forces: np.ndarray):
        """
        Converts thruster forces into PWM signals and publishes them.
        Applies individual limits to prevent overcurrent.
        """

        pwm_arr = [force_to_pwm_thruster(i + 1, float(thrust_forces[i])) for i in range(8)]

        # Apply limit checking for each thruster
        pwm_arr = np.clip(pwm_arr, self.thruster_lower_limit, self.thruster_upper_limit)
        pwm_arr = pwm_arr.astype(np.uint16, copy=False) # Ensure uint16 for message compatibility
        
        pwm_msg = Int16MultiArray()
        pwm_msg.data = pwm_arr.tolist()
        self.pub_us.publish(pwm_msg)

    def re_arm(self):
        """
        Sends the arming signal to the thrusters upon startup.
        """
        msg1 = Int16MultiArray(data=[1500] * 8)
        msg2 = Int16MultiArray(data=[1540] * 8)
        self.pub_us.publish(msg1)
        time.sleep(0.5)
        self.pub_us.publish(msg2)
        time.sleep(3.0)
        self.pub_us.publish(msg1)

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
        # Ensure safe shutdown of thrusters
        try:
            thrust_mapper.shutdown_thrusters()
        except Exception:
            pass
        thrust_mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
