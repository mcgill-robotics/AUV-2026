#!/usr/bin/env python3
"""
Description: Thrust mapper node subscribes to the effort topic, converts the wrench readings to thruster forces,
and then converts the forces to PWM signals and publishes them.

Uses direction-preserving, attitude-priority thrust allocation:
  1. Attitude (torques) are allocated first and scaled to stay within f_max per thruster.
  2. Translation (forces) fill remaining thruster capacity without corrupting attitude.
  3. All scaling preserves the direction of the original wrench vector.
This eliminates the "death spin" caused by naive PWM clipping during saturation.

Subscribes:  /controls/total_effort (geometry_msgs/msg/Wrench)
Publishes:   /propulsion/microseconds (std_msgs/msg/Int16MultiArray)
             /propulsion/forces      (auv_msgs/msg/ThrusterForces)
Parameters:  a, b, c, d, e, dx, dy (m), alpha (deg), f_max (N),
             thruster_PWM_lower_limit, thruster_PWM_upper_limit
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

        # --- Force limit per thruster (Newtons)
        # Conservative default below the ~22 N effective limit imposed by PWM window [1228, 1768].
        # Increase via launch param if more thrust is needed; the PWM clip is a safety backstop.
        self.declare_parameter('f_max', 20.0)

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
            self.f_max = float(self.get_parameter('f_max').value)

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

        self.get_logger().info(f'Priority allocator: f_max = {self.f_max:.1f} N per thruster')

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

        # Throttle logging: only log scaling events every N callbacks
        self._log_counter = 0
        self._LOG_INTERVAL = 50  # log every 50th scaled callback (~1.7s at 30 Hz)

        # --- Subscriber
        self.sub_cmd = self.create_subscription(
            Wrench,
            'controls/total_effort',
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


    def _allocate_priority(self, wrench_msg: Wrench) -> np.ndarray:
        """
        Direction-preserving, three-tier priority thrust allocation.

        Priority order (highest to lowest):
          1. Attitude (roll, pitch, yaw torques) - for stability.
          2. Depth (heave force Fz) - maintaining depth.
          3. Planar (surge Fx, sway Fy) - for speed.

        Each tier is allocated via pseudo-inverse, direction-preserve scaled if
        it alone exceeds f_max, then fit into the remaining capacity left by
        all higher tiers using sign-aware alpha computation.

        Returns the 8-element thrust force vector in Newtons.
        """
        f_max = self.f_max

        # --- Split wrench into three priority tiers ---
        tau_att = np.array([
            0.0, 0.0, 0.0,
            wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z
        ])
        tau_depth = np.array([
            0.0, 0.0, wrench_msg.force.z,
            0.0, 0.0, 0.0
        ])
        tau_planar = np.array([
            wrench_msg.force.x, wrench_msg.force.y, 0.0,
            0.0, 0.0, 0.0
        ])

        # --- Tier 1: Attitude (highest priority) ---
        f_att = self.T_inv @ tau_att
        att_max = np.max(np.abs(f_att))
        att_scale = att_max / f_max if att_max > f_max else 1.0
        if att_scale > 1.0:
            f_att /= att_scale

        # --- Tier 2: Depth (mid priority) ---
        f_depth = self.T_inv @ tau_depth
        alpha_depth = self._compute_max_translation_alpha(f_att, f_depth, f_max)
        f_baseline = f_att + alpha_depth * f_depth

        # --- Tier 3: Planar (lowest priority) ---
        f_planar = self.T_inv @ tau_planar
        alpha_planar = self._compute_max_translation_alpha(f_baseline, f_planar, f_max)

        # --- Combine all tiers ---
        thrust_forces = f_baseline + alpha_planar * f_planar

        # --- Periodic logging ---
        self._log_counter += 1
        if (att_scale > 1.0 or alpha_depth < 1.0 or alpha_planar < 1.0) \
                and self._log_counter >= self._LOG_INTERVAL:
            self._log_counter = 0
            self.get_logger().info(
                f'Allocation scaled: att={att_scale:.2f}, '
                f'depth_alpha={alpha_depth:.2f}, planar_alpha={alpha_planar:.2f}'
            )

        return thrust_forces

    @staticmethod
    def _compute_max_translation_alpha(
        f_baseline: np.ndarray, f_new: np.ndarray, f_max: float
    ) -> float:
        """
        Compute the maximum scaling factor alpha in [0, 1] such that
        |f_baseline[i] + alpha * f_new[i]| <= f_max for every thruster i.

        Used to layer each priority tier on top of the accumulated baseline
        from all higher-priority tiers (e.g. attitude, then depth, then planar).

        Sign-aware: if baseline and new forces oppose each other on a thruster,
        the cancellation is accounted for, avoiding overly conservative scaling.

        Args:
            f_baseline: Per-thruster forces already committed by higher-priority tiers.
            f_new:      Per-thruster forces for the current tier to be scaled in.
            f_max:      Absolute force limit per thruster (Newtons).

        Returns:
            alpha in [0, 1] - the fraction of f_new that fits without exceeding f_max.
        """
        alpha = 1.0
        for i in range(len(f_new)):
            ft = f_new[i]
            if abs(ft) < 1e-9:
                continue

            # Determine the binding constraint depending on which limit
            # f_new pushes toward
            if ft > 0.0:
                # Pushing toward +f_max
                alpha_i = (f_max - f_baseline[i]) / ft
            else:
                # Pushing toward -f_max
                alpha_i = (-f_max - f_baseline[i]) / ft

            if alpha_i < alpha:
                alpha = alpha_i

        # Clamp to [0, 1] - negative alpha means the baseline already uses
        # all capacity in the direction f_new wants to push.
        return max(0.0, min(1.0, alpha))

    def wrench_to_thrust(self, wrench_msg: Wrench):
        """
        Callback: maps a Wrench into per-thruster forces via priority allocation,
        then publishes forces and PWM.
        """
        # Priority allocation in force space
        thrust_forces = self._allocate_priority(wrench_msg)

        # Publish forces (debug/telemetry)
        tf_msg = ThrusterForces()
        tf_msg.back_right         = float(thrust_forces[0])
        tf_msg.heave_back_right    = float(thrust_forces[1])
        tf_msg.heave_front_right   = float(thrust_forces[2])
        tf_msg.front_right       = float(thrust_forces[3])
        tf_msg.front_left         = float(thrust_forces[4])
        tf_msg.heave_front_left    = float(thrust_forces[5])
        tf_msg.heave_back_left    = float(thrust_forces[6])
        tf_msg.back_left         = float(thrust_forces[7])
        self.pub_forces.publish(tf_msg)

        # Convert to PWM and publish
        self.forces_to_pwm_publisher(thrust_forces)

    def forces_to_pwm_publisher(self, thrust_forces: np.ndarray):
        """
        Converts thruster forces into PWM signals and publishes them.
        The PWM clip is a safety backstop only - the priority allocator should
        keep forces within f_max so this clip never activates in normal operation.
        """
        pwm_arr = np.array(
            [force_to_pwm_thruster(i + 1, float(thrust_forces[i])) for i in range(8)],
            dtype=float
        )

        # Safety backstop clip - warn if it activates
        clipped = np.clip(pwm_arr, self.thruster_lower_limit, self.thruster_upper_limit)
        if not np.array_equal(pwm_arr, clipped):
            self.get_logger().warn(
                'PWM safety clip activated! f_max may be set too high. '
                f'Pre-clip: {pwm_arr.astype(int).tolist()}, '
                f'limits: [{self.thruster_lower_limit}, {self.thruster_upper_limit}]'
            )
        pwm_arr = clipped.astype(np.uint16, copy=False)

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
