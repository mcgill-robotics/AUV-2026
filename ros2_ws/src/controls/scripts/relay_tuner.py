#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import Wrench, PointStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu


def quat_to_euler_deg(q):
    """Convert quaternion orientation to roll, pitch, yaw in degrees."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(90.0, sinp)
    else:
        pitch = math.asin(sinp) * (180.0 / math.pi)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

    return roll, pitch, yaw


class RelayTunerNode(Node):
    def __init__(self):
        super().__init__('relay_tuner')

        # 1. Tuner Configuration & Parameters
        self.declare_parameter('tune_axis', 'depth')
        self.declare_parameter('setpoint', 1.0)
        self.declare_parameter('relay_amp_pos', 15.0)
        self.declare_parameter('relay_amp_neg', -15.0)
        self.declare_parameter('base_effort', 0.0)
        self.declare_parameter('hysteresis_band', 0.05)
        self.declare_parameter('max_bounds', 0.0)
        self.declare_parameter('num_cycles', 5)
        self.declare_parameter('min_warmup_cycles', 1)
        self.declare_parameter('max_warmup_cycles', 6)
        self.declare_parameter('convergence_tol', 0.10)
        self.declare_parameter('control_loop_hz', 50.0)

        self.tune_axis = str(self.get_parameter('tune_axis').value).lower()
        self.setpoint = float(self.get_parameter('setpoint').value)
        self.relay_amp_pos = float(self.get_parameter('relay_amp_pos').value)
        self.relay_amp_neg = float(self.get_parameter('relay_amp_neg').value)
        self.base_effort = float(self.get_parameter('base_effort').value)
        self.hysteresis_band = float(self.get_parameter('hysteresis_band').value)
        self.max_bounds = float(self.get_parameter('max_bounds').value)
        self.num_cycles = int(self.get_parameter('num_cycles').value)
        self.min_warmup_cycles = int(self.get_parameter('min_warmup_cycles').value)
        self.max_warmup_cycles = int(self.get_parameter('max_warmup_cycles').value)
        self.convergence_tol = float(self.get_parameter('convergence_tol').value)
        self.control_loop_hz = float(self.get_parameter('control_loop_hz').value)

        # Axis Mapping
        if self.tune_axis in ['depth', 'z']:
            self.target_controller = 'depth_controller'
            self.effort_topic = '/controls/depth_effort'
            self.state_topic = 'auv_frame/depth'
            self.is_angle = False
        elif self.tune_axis in ['pitch', 'roll', 'yaw']:
            self.target_controller = 'attitude_controller'
            self.effort_topic = '/controls/attitude_effort'
            self.state_topic = 'auv_frame/imu'
            self.is_angle = True
        elif self.tune_axis in ['surge', 'x']:
            self.target_controller = 'x_controller'
            self.effort_topic = '/controls/x_effort'
            self.state_topic = 'auv_frame/dvl/position'
            self.is_angle = False
        elif self.tune_axis in ['sway', 'y']:
            self.target_controller = 'y_controller'
            self.effort_topic = '/controls/y_effort'
            self.state_topic = 'auv_frame/dvl/position'
            self.is_angle = False
        else:
            raise ValueError(f"Unsupported tune_axis: {self.tune_axis}. Must be one of: depth, pitch, roll, yaw, surge, sway, x, y.")

        if self.max_bounds <= 0.0:
            self.max_bounds = 20.0 if self.is_angle else 1.0

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers & Subscribers
        self.pub_effort = self.create_publisher(Wrench, self.effort_topic, qos_profile_sensor_data)

        if self.state_topic == 'auv_frame/depth':
            self.sub_state = self.create_subscription(Float64, self.state_topic, self.depth_callback, qos_profile_sensor_data)
        elif self.state_topic == 'auv_frame/imu':
            self.sub_state = self.create_subscription(Imu, self.state_topic, self.imu_callback, qos_profile_sensor_data)
        elif self.state_topic == 'auv_frame/dvl/position':
            self.sub_state = self.create_subscription(PointStamped, self.state_topic, self.position_callback, qos_profile_sensor_data)

        # Parameter service client to disable/enable target controller
        self.param_client = self.create_client(SetParameters, f'/{self.target_controller}/set_parameters')

        # State Machine Variables
        self.state = 'INIT'
        self.current_state_val = None
        self.current_relay_output = 0.0
        self.cycle_count = 0
        self.warmup_count = 0
        self.is_warmup = True
        self.initial_error = 0.0
        self.last_warmup_amp = None
        self.last_warmup_period = None
        self.switch_times = []
        self.cycle_amplitudes = []
        self.cycle_max = -float('inf')
        self.cycle_min = float('inf')
        self.last_switch_state = None  # 'pos' or 'neg'
        self.disable_future = None
        self.disable_start_time = None

        self.time_step = 1.0 / self.control_loop_hz
        self.timer = self.create_timer(self.time_step, self.control_loop_callback)

        self.get_logger().info(
            f"Initialized Topside Relay Tuner for axis '{self.tune_axis}' | "
            f"Setpoint: {self.setpoint} | Relay Amps: [{self.relay_amp_neg}, +{self.relay_amp_pos}] | "
            f"Base Effort: {self.base_effort} | Hysteresis: {self.hysteresis_band} | Max Bounds: {self.max_bounds} | "
            f"Adaptive Warmup: [{self.min_warmup_cycles}-{self.max_warmup_cycles} cycles, tol: {self.convergence_tol*100:.0f}%] | Measurement Cycles: {self.num_cycles}"
        )

    def depth_callback(self, msg):
        self.current_state_val = msg.data

    def imu_callback(self, msg):
        roll, pitch, yaw = quat_to_euler_deg(msg.orientation)
        if self.tune_axis == 'roll':
            self.current_state_val = roll
        elif self.tune_axis == 'pitch':
            self.current_state_val = pitch
        elif self.tune_axis == 'yaw':
            self.current_state_val = yaw

    def position_callback(self, msg):
        if self.tune_axis in ['surge', 'x']:
            self.current_state_val = msg.point.x
        elif self.tune_axis in ['sway', 'y']:
            self.current_state_val = msg.point.y

    def set_controller_enabled_async(self, enabled_val: bool):
        if not self.param_client.service_is_ready():
            self.get_logger().warn(f"Service /{self.target_controller}/set_parameters not ready. Assuming controller is already in desired state.")
            return None
        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled_val)
        req.parameters = [Parameter(name='enabled', value=val)]
        return self.param_client.call_async(req)

    def publish_effort(self, effort_val):
        msg = Wrench()
        if self.tune_axis in ['depth', 'z']:
            msg.force.z = effort_val
        elif self.tune_axis in ['surge', 'x']:
            msg.force.x = effort_val
        elif self.tune_axis in ['sway', 'y']:
            msg.force.y = effort_val
        elif self.tune_axis == 'roll':
            msg.torque.x = effort_val
        elif self.tune_axis == 'pitch':
            msg.torque.y = effort_val
        elif self.tune_axis == 'yaw':
            msg.torque.z = effort_val
        self.pub_effort.publish(msg)

    def publish_zero_effort(self):
        self.publish_effort(0.0)

    def control_loop_callback(self):
        if self.current_state_val is None:
            return

        now = time.time()

        # Phase 1: Disable target controller
        if self.state == 'INIT':
            self.get_logger().info(f"Disabling target controller '{self.target_controller}'...")
            self.disable_future = self.set_controller_enabled_async(False)
            self.disable_start_time = now
            self.state = 'WAITING_DISABLE'
            return

        if self.state == 'WAITING_DISABLE':
            if self.disable_future is None or self.disable_future.done() or (now - self.disable_start_time > 3.0):
                self.get_logger().info("Target controller disabled. Starting Bang-Bang Relay Loop!")
                self.initial_error = abs(self.current_state_val - self.setpoint)
                if self.current_state_val < self.setpoint:
                    self.current_relay_output = self.relay_amp_pos
                    self.last_switch_state = 'pos'
                else:
                    self.current_relay_output = self.relay_amp_neg
                    self.last_switch_state = 'neg'
                self.cycle_max = self.current_state_val
                self.cycle_min = self.current_state_val
                self.state = 'RELAY_LOOP'
            return

        # Phase 2: Bang-Bang Relay Limit-Cycle Loop
        if self.state == 'RELAY_LOOP':
            # 5. Safety Constraints: Check max bounds cutoff (allow approach corridor during warmup if started away from setpoint)
            error = abs(self.current_state_val - self.setpoint)
            effective_max_bounds = max(self.max_bounds, self.initial_error + (5.0 if self.is_angle else 0.5)) if self.is_warmup else self.max_bounds
            if error > effective_max_bounds:
                self.get_logger().error(
                    f"CRITICAL SAFETY CUTOFF: State error ({error:.3f}) exceeds effective max_bounds ({effective_max_bounds:.3f})! "
                    "Relay is not strong enough to reverse momentum or drift. Aborting test."
                )
                self.publish_zero_effort()
                self.set_controller_enabled_async(True)
                self.state = 'ABORTED'
                return

            # Track Peak Extremes
            if self.current_state_val > self.cycle_max:
                self.cycle_max = self.current_state_val
            if self.current_state_val < self.cycle_min:
                self.cycle_min = self.current_state_val

            # Hysteresis Relay Switch Logic
            if self.current_state_val < self.setpoint - self.hysteresis_band and self.last_switch_state != 'pos':
                self.last_switch_state = 'pos'
                self.current_relay_output = self.relay_amp_pos
            elif self.current_state_val > self.setpoint + self.hysteresis_band and self.last_switch_state != 'neg':
                self.last_switch_state = 'neg'
                self.current_relay_output = self.relay_amp_neg
                self.switch_times.append(now)

                if len(self.switch_times) > 1:
                    cycle_period = self.switch_times[-1] - self.switch_times[-2]
                    amp = (self.cycle_max - self.cycle_min) / 2.0

                    if self.is_warmup:
                        self.warmup_count += 1
                        converged = False
                        if self.last_warmup_amp is not None and self.last_warmup_period is not None:
                            amp_diff_rel = abs(amp - self.last_warmup_amp) / max(1e-4, self.last_warmup_amp)
                            per_diff_rel = abs(cycle_period - self.last_warmup_period) / max(1e-4, self.last_warmup_period)
                            amp_diff_abs = abs(amp - self.last_warmup_amp)
                            per_diff_abs = abs(cycle_period - self.last_warmup_period)

                            # Stable if relative change < convergence_tol OR absolute change within sensor noise floor
                            amp_stable = (amp_diff_rel < self.convergence_tol) or (amp_diff_abs < (0.5 if self.is_angle else 0.02))
                            per_stable = (per_diff_rel < self.convergence_tol) or (per_diff_abs < 0.10)

                            self.get_logger().info(
                                f"--- [ADAPTIVE WARMUP {self.warmup_count}/{self.max_warmup_cycles}] --- | "
                                f"Period: {cycle_period:.3f}s (Δ: {per_diff_rel*100:.1f}%) | Amp: {amp:.3f} (Δ: {amp_diff_rel*100:.1f}%) | "
                                f"Extremes: [{self.cycle_min:.3f}, {self.cycle_max:.3f}]"
                            )

                            if self.warmup_count >= self.min_warmup_cycles and amp_stable and per_stable:
                                converged = True
                                self.get_logger().info(
                                    f"⚡ LIMIT CYCLE CONVERGED on Warmup Cycle {self.warmup_count}! "
                                    f"(Δa: {amp_diff_rel*100:.1f}%, ΔP: {per_diff_rel*100:.1f}% < {self.convergence_tol*100:.0f}% tolerance)."
                                )
                        else:
                            self.get_logger().info(
                                f"--- [ADAPTIVE WARMUP {self.warmup_count}/{self.max_warmup_cycles}] --- | "
                                f"Period: {cycle_period:.3f}s | Amp: {amp:.3f} | Extremes: [{self.cycle_min:.3f}, {self.cycle_max:.3f}] "
                                "(Establishing initial baseline)"
                            )

                        self.last_warmup_amp = amp
                        self.last_warmup_period = cycle_period

                        if not converged and self.warmup_count >= self.max_warmup_cycles:
                            self.get_logger().warn(
                                f"Reached max warmup cycles ({self.max_warmup_cycles}) without meeting strict {self.convergence_tol*100:.0f}% tolerance. "
                                "Proceeding to steady-state measurement phase."
                            )
                            converged = True

                        if converged:
                            self.is_warmup = False
                            self.get_logger().info("Limit cycle stabilized. Beginning steady-state measurement cycles...")
                            # Keep only the last timestamp as start time t0 for steady-state measurement
                            self.switch_times = [self.switch_times[-1]]
                            self.cycle_amplitudes.clear()
                            self.cycle_count = 0
                    else:
                        self.cycle_count += 1
                        self.cycle_amplitudes.append(amp)
                        self.get_logger().info(
                            f"--- [MEASUREMENT CYCLE {self.cycle_count}/{self.num_cycles}] --- | "
                            f"Period (P_u): {cycle_period:.3f}s | Amplitude (a): {amp:.3f} | "
                            f"Extremes: [{self.cycle_min:.3f}, {self.cycle_max:.3f}]"
                        )
                        # 3. Termination & Calculation
                        if self.cycle_count >= self.num_cycles:
                            self.get_logger().info(f"{self.num_cycles} steady-state cycles recorded! Terminating relay loop...")
                            self.publish_zero_effort()
                            self.set_controller_enabled_async(True)
                            self.state = 'CALCULATING'
                            self.calculate_and_print_gains()
                            return

                    # Reset peak tracking for next oscillation cycle
                    self.cycle_max = self.current_state_val
                    self.cycle_min = self.current_state_val

            # Publish active effort (base_effort + relay output)
            self.publish_effort(self.base_effort + self.current_relay_output)
            return

        if self.state in ['CALCULATING', 'ABORTED']:
            self.publish_zero_effort()

    def calculate_and_print_gains(self):
        periods = [self.switch_times[i] - self.switch_times[i - 1] for i in range(1, len(self.switch_times))]
        P_u = float(np.mean(periods))
        a = float(np.mean(self.cycle_amplitudes))
        d = (abs(self.relay_amp_pos) + abs(self.relay_amp_neg)) / 2.0

        if self.is_angle:
            # Native Quaternion State-Space Transformation (sin(theta / 2))
            # Converts degrees directly to C++ attitude_controller error space!
            a_nat = math.sin(a * math.pi / 360.0)
            eps_nat = math.sin(self.hysteresis_band * math.pi / 360.0)

            if a_nat <= eps_nat:
                self.get_logger().warn(f"Native amplitude a_nat ({a_nat:.4f}) <= eps_nat ({eps_nat:.4f}). Clamping denominator.")
                denom = 1e-4
            else:
                denom = math.pi * math.sqrt(a_nat**2 - eps_nat**2)

            K_u = (4.0 * d) / denom  # Natively in [Nm / quaternion_error]

            # AUV-Specific High-Stiffness PD Rules (Double-Integrator with Drag & Latency)
            kp_auv = 0.60 * K_u
            kd_auv = 0.15 * K_u * P_u  # Natively in [Nm / (rad/s)] because P_u is in seconds!
            ki_auv = 0.0
        else:
            if a <= self.hysteresis_band:
                self.get_logger().warn(f"Average amplitude a ({a:.4f}) <= hysteresis band ({self.hysteresis_band:.4f}). Clamping denominator.")
                denom = 1e-4
            else:
                denom = math.pi * math.sqrt(a**2 - self.hysteresis_band**2)

            K_u = (4.0 * d) / denom  # In [N / m]

            # AUV-Specific High-Stiffness PD Rules
            kp_auv = 0.60 * K_u
            kd_auv = 0.15 * K_u * P_u
            ki_auv = 0.0

        # Rule A: Tyreus-Luyben (for comparison)
        kp_tl = 0.45 * K_u
        ki_tl = kp_tl / (2.2 * P_u)
        kd_tl = kp_tl * (P_u / 6.3)

        # Rule B: No-Overshoot (for comparison)
        kp_no = 0.2 * K_u
        ki_no = kp_no / (0.4 * P_u)
        kd_no = kp_no * (0.066 * P_u)

        print("\n" + "=" * 70)
        print(f"TOPSIDE RELAY TUNER RESULTS ({self.tune_axis.upper()} AXIS)")
        print("=" * 70)
        print(f"Ultimate Gain (K_u):   {K_u:.4f} {'[Nm/quat_error]' if self.is_angle else '[N/m]'}")
        print(f"Ultimate Period (P_u): {P_u:.4f} s")
        print(f"Average Amplitude (a): {a:.4f} {'deg' if self.is_angle else 'm'}")
        print(f"Drive Amplitude (d):   {d:.4f} {'Nm' if self.is_angle else 'N'}")
        print(f"Hysteresis Band (eps): {self.hysteresis_band:.4f} {'deg' if self.is_angle else 'm'}")
        print("-" * 70)
        print("RECOMMENDED: AUV-Specific High-Stiffness PD Rules (Snappy, No Hunting):")
        print(f"  KP: {kp_auv:.4f}")
        print(f"  KI: {ki_auv:.4f}")
        print(f"  KD: {kd_auv:.4f}")
        print("-" * 70)
        print("Rule A: Tyreus-Luyben (Conservative):")
        print(f"  KP: {kp_tl:.4f} | KI: {ki_tl:.4f} | KD: {kd_tl:.4f}")
        print("Rule B: No-Overshoot (Aggressive Braking):")
        print(f"  KP: {kp_no:.4f} | KI: {ki_no:.4f} | KD: {kd_no:.4f}")
        print("=" * 70)

        if self.is_angle:
            axis_map = {
                'roll':  ('P_ex', 'I_ex', 'P_wx'),
                'pitch': ('P_ey', 'I_ey', 'P_wy'),
                'yaw':   ('P_ez', 'I_ez', 'P_wz')
            }
            p_name, i_name, d_name = axis_map.get(self.tune_axis, ('KP', 'KI', 'KD'))

            print("\nYAML Copy-Paste Block for attitude_controller (Natively Scaled for Quaternions & rad/s):")
            print("-----------------------------------------------------------------------------------------")
            print(f"# Topside Relay Tuner ({self.tune_axis}) - AUV-Specific High-Stiffness PD (RECOMMENDED)")
            print(f"{p_name}: {kp_auv:.4f}  # 0.60 * K_u (Native Quaternion Space)")
            print(f"{i_name}: {ki_auv:.4f}  # Zeroed to prevent integral hunting")
            print(f"{d_name}: {kd_auv:.4f}  # 0.15 * K_u * P_u (Native rad/s Damping)")
            print("-----------------------------------------------------------------------------------------\n")
        else:
            print("\nYAML Copy-Paste Block for Controller_params_real.yaml:")
            print("------------------------------------------------------")
            print(f"# Topside Relay Tuner ({self.tune_axis}) - AUV-Specific High-Stiffness PD")
            print(f"KP: {kp_auv:.4f}")
            print(f"KI: {ki_auv:.4f}")
            print(f"KD: {kd_auv:.4f}")
            print("------------------------------------------------------\n")


def main():
    rclpy.init()
    node = RelayTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Relay tuner interrupted by user.")
        node.publish_zero_effort()
        node.set_controller_enabled_async(True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
