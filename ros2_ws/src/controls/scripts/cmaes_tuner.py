#!/usr/bin/env python3
"""
SITL Automated PID Tuning Pipeline using CMA-ES (Covariance Matrix Adaptation Evolution Strategy).
Designed for Unity marine robotics simulation running in fast-forward (5x-10x) with ROS 2 Humble.
"""

import os
import time
import math
import threading
import numpy as np
import cma
import yaml
import rclpy
import argparse

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Wrench, PointStamped, Quaternion
from sensor_msgs.msg import Imu
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from scipy.spatial.transform import Rotation
from controls.utils import quaternion_to_euler


class CMAESTunerNode(Node):
    def __init__(self, tune_mode=None):
        super().__init__('cmaes_tuner')

        # ---------------------------------------------------------
        # REQUIREMENT 4: Simulation Time Adherence
        # ---------------------------------------------------------
        # Ensure use_sim_time is declared and enabled by default
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self.get_logger().info(f"Initialized CMA-ES Tuner Node (use_sim_time={self.use_sim_time})")

        # ---------------------------------------------------------
        # Tuner Configuration Parameters & Cascaded Schedule Mode
        # ---------------------------------------------------------
        self.declare_parameter('tune_mode', '')               # attitude, depth, planar
        self.declare_parameter('target_controller', '/depth_controller')
        self.declare_parameter('target_setpoint', 0.25)       # Target depth/position/angle
        self.declare_parameter('randomize_setpoints', True)   # Randomize setpoint per generation
        self.declare_parameter('sim_duration_sec', 10.0)      # Simulated evaluation window per candidate
        self.declare_parameter('popsize', 8)                  # CMA-ES population size per generation
        self.declare_parameter('maxiter', 15)                 # Max CMA-ES generations
        
        # Parameter mapping
        self.declare_parameter('param_name_kp', 'KP')
        self.declare_parameter('param_name_ki', 'KI')
        self.declare_parameter('param_name_kd', 'KD')

        # Topic and axis mapping
        self.declare_parameter('state_topic', 'auv_frame/depth')
        self.declare_parameter('effort_topic', '/controls/depth_effort')
        self.declare_parameter('effort_axis', 'force_z')      # force_x, force_y, force_z, torque_x, torque_y, torque_z

        # Initial PID gain guesses and step size
        self.declare_parameter('init_kp', 50.0)
        self.declare_parameter('init_ki', 5.0)
        self.declare_parameter('init_kd', 20.0)
        self.declare_parameter('init_sigma', 15.0)            # Initial CMA-ES exploration step size

        # Cost function weights and thresholds
        self.declare_parameter('weight_effort', 0.0001)       # Penalty multiplier for tau^2
        self.declare_parameter('overshoot_threshold', 0.02)   # Tolerance before overshoot penalty triggers
        self.declare_parameter('overshoot_floor_deg', 1.5)    # Absolute minimum overshoot tolerance for angular controllers (degrees)
        self.declare_parameter('overshoot_floor_meters', 0.05)# Absolute minimum overshoot tolerance for translational controllers (meters)
        self.declare_parameter('overshoot_penalty_base', 5000.0)

        # Load values
        self.target_controller = str(self.get_parameter('target_controller').value)
        self.target_setpoint = float(self.get_parameter('target_setpoint').value)
        self.randomize_setpoints = bool(self.get_parameter('randomize_setpoints').value)
        self.current_eval_setpoint = self.target_setpoint
        self.sim_duration_sec = float(self.get_parameter('sim_duration_sec').value)
        self.popsize = int(self.get_parameter('popsize').value)
        self.maxiter = int(self.get_parameter('maxiter').value)
        
        self.param_name_kp = str(self.get_parameter('param_name_kp').value)
        self.param_name_ki = str(self.get_parameter('param_name_ki').value)
        self.param_name_kd = str(self.get_parameter('param_name_kd').value)

        self.state_topic = str(self.get_parameter('state_topic').value)
        self.effort_topic = str(self.get_parameter('effort_topic').value)
        self.effort_axis = str(self.get_parameter('effort_axis').value)
        self.setpoint_topic = f"{self.target_controller.replace('_controller', '')}_setpoint"
        if not self.setpoint_topic.startswith('/controls/'):
            self.setpoint_topic = f"/controls/{self.setpoint_topic.lstrip('/')}"

        # ---------------------------------------------------------
        # Load Tuner Configuration & Mode Definitions from YAML
        # ---------------------------------------------------------
        modes_cfg = self.load_tuner_config_from_yaml()
        active_mode = (tune_mode or str(self.get_parameter('tune_mode').value)).lower()
        self.auxiliary_controllers = []

        mode_key = active_mode
        if active_mode in ['attitude', 'attitude_pitch', 'pitch']: mode_key = 'pitch'
        elif active_mode in ['attitude_roll', 'roll']: mode_key = 'roll'
        elif active_mode in ['attitude_yaw', 'yaw']: mode_key = 'yaw'
        elif active_mode in ['heave', 'depth']: mode_key = 'depth'
        elif active_mode in ['planar', 'surge', 'planar_x']: mode_key = 'surge'
        elif active_mode in ['planar_y', 'sway']: mode_key = 'sway'
        elif active_mode in ['full', 'joint', 'all']: mode_key = 'full'

        if mode_key in modes_cfg:
            cfg = modes_cfg[mode_key]
            self.get_logger().info(f"CASCADED SCHEDULE: TUNING MODE '{mode_key.upper()}' (Loaded from YAML)")
            self.is_multi_controller = bool(cfg.get('is_multi_controller', False))
            self.target_controller = cfg.get('target_controller', self.target_controller)
            self.param_name_kp = cfg.get('param_name_kp', self.param_name_kp)
            self.param_name_ki = cfg.get('param_name_ki', self.param_name_ki)
            self.param_name_kd = cfg.get('param_name_kd', self.param_name_kd)
            self.state_topic = cfg.get('state_topic', self.state_topic)
            self.effort_topic = cfg.get('effort_topic', self.effort_topic)
            self.effort_axis = cfg.get('effort_axis', self.effort_axis)
            self.setpoint_topic = cfg.get('setpoint_topic', self.setpoint_topic)
            self.auxiliary_controllers = list(cfg.get('auxiliary_controllers', []))
            
            if self.target_setpoint == 0.25 and 'default_setpoint' in cfg:
                self.target_setpoint = cfg['default_setpoint']
            if 'setpoint_min' in cfg and 'setpoint_max' in cfg:
                self.setpoint_range = (cfg['setpoint_min'], cfg['setpoint_max'])
            if 'default_angular_setpoint' in cfg:
                self.default_angular_setpoint = cfg['default_angular_setpoint']
            else:
                self.default_angular_setpoint = [10.0, 15.0, 30.0]
            if 'angular_setpoint_min' in cfg and 'angular_setpoint_max' in cfg:
                self.angular_setpoint_range = (cfg['angular_setpoint_min'], cfg['angular_setpoint_max'])
            if 'init_sigma' in cfg:
                self.init_sigma_default = float(cfg['init_sigma'])
        elif active_mode:
            self.get_logger().warn(f"Unknown tune mode '{active_mode}', using CLI parameters.")

        self.init_kp = float(self.get_parameter('init_kp').value)
        self.init_ki = float(self.get_parameter('init_ki').value)
        self.init_kd = float(self.get_parameter('init_kd').value)
        self.init_sigma = float(self.get_parameter('init_sigma').value)
        if self.init_sigma == 15.0 and hasattr(self, 'init_sigma_default'):
            self.init_sigma = self.init_sigma_default

        # Auto-load initial PID guesses from Controller_params_sim.yaml
        self.load_initial_guesses_from_yaml()

        self.weight_effort = float(self.get_parameter('weight_effort').value)
        self.overshoot_threshold = float(self.get_parameter('overshoot_threshold').value)
        self.overshoot_floor_deg = float(self.get_parameter('overshoot_floor_deg').value)
        self.overshoot_floor_meters = float(self.get_parameter('overshoot_floor_meters').value)
        self.overshoot_penalty_base = float(self.get_parameter('overshoot_penalty_base').value)

        # Reentrant callback group allows concurrent subscription processing and parameter service calls
        self.cb_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------
        # Publishers, Subscribers, and Clients
        # ---------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Reset publisher to Unity physics engine
        self.pub_reset = self.create_publisher(Bool, '/simulation/reset', qos, callback_group=self.cb_group)
        
        # Setpoint publishers and Parameter clients
        if getattr(self, 'is_multi_controller', False):
            self.pub_sp_quat = self.create_publisher(Quaternion, '/controls/quaternion_setpoint', qos, callback_group=self.cb_group)
            self.pub_sp_depth = self.create_publisher(Float64, '/controls/depth_setpoint', qos, callback_group=self.cb_group)
            self.pub_sp_x = self.create_publisher(Float64, '/controls/x_setpoint', qos, callback_group=self.cb_group)
            self.pub_sp_y = self.create_publisher(Float64, '/controls/y_setpoint', qos, callback_group=self.cb_group)

            self.sub_state_att = self.create_subscription(Imu, 'auv_frame/imu', self.state_cb_att, qos_profile_sensor_data, callback_group=self.cb_group)
            self.sub_state_depth = self.create_subscription(Float64, 'auv_frame/depth', self.state_cb_depth, qos_profile_sensor_data, callback_group=self.cb_group)
            self.sub_state_pos = self.create_subscription(PointStamped, 'auv_frame/dvl/position', self.state_cb_pos, qos_profile_sensor_data, callback_group=self.cb_group)
            
            self.sub_eff_att = self.create_subscription(Wrench, '/controls/attitude_effort', self.effort_cb_multi, qos_profile_sensor_data, callback_group=self.cb_group)
            self.sub_eff_depth = self.create_subscription(Wrench, '/controls/depth_effort', self.effort_cb_multi, qos_profile_sensor_data, callback_group=self.cb_group)
            self.sub_eff_x = self.create_subscription(Wrench, '/controls/x_effort', self.effort_cb_multi, qos_profile_sensor_data, callback_group=self.cb_group)
            self.sub_eff_y = self.create_subscription(Wrench, '/controls/y_effort', self.effort_cb_multi, qos_profile_sensor_data, callback_group=self.cb_group)
        else:
            if self.setpoint_topic == '/controls/quaternion_setpoint':
                self.pub_setpoint = self.create_publisher(Quaternion, self.setpoint_topic, qos, callback_group=self.cb_group)
            else:
                self.pub_setpoint = self.create_publisher(Float64, self.setpoint_topic, qos, callback_group=self.cb_group)

            param_service_name = f"{self.target_controller}/set_parameters"
            self.param_client = self.create_client(SetParameters, param_service_name, callback_group=self.cb_group)

            if self.state_topic == 'auv_frame/imu':
                self.sub_state = self.create_subscription(
                    Imu, self.state_topic, self.state_callback, qos_profile_sensor_data, callback_group=self.cb_group
                )
            elif self.state_topic == 'auv_frame/dvl/position':
                self.sub_state = self.create_subscription(
                    PointStamped, self.state_topic, self.state_callback, qos_profile_sensor_data, callback_group=self.cb_group
                )
            else:
                self.sub_state = self.create_subscription(
                    Float64, self.state_topic, self.state_callback, qos_profile_sensor_data, callback_group=self.cb_group
                )

            self.sub_effort = self.create_subscription(
                Wrench, self.effort_topic, self.effort_callback, qos_profile_sensor_data, callback_group=self.cb_group
            )

        # ---------------------------------------------------------
        # Thread-safe Recording Buffer
        # ---------------------------------------------------------
        self.lock = threading.Lock()
        self.is_recording = False
        self.state_history = []   # List of (sim_time_sec, state_val)
        self.effort_history = []  # List of (sim_time_sec, effort_val)
        self.multi_state = {'roll': [], 'pitch': [], 'yaw': [], 'depth': [], 'x': [], 'y': []}
        self.multi_effort = []

        # Start CMA-ES optimization in a background worker thread so ROS executor spins cleanly
        self.tuner_thread = threading.Thread(target=self.run_optimization, daemon=True)
        self.tuner_thread.start()

    def state_callback(self, msg):
        if self.is_recording:
            # Must use simulation clock (/clock) from Unity
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            val = 0.0
            if hasattr(msg, 'orientation'):
                roll_rad, pitch_rad, yaw_rad = quaternion_to_euler(msg.orientation)
                if self.effort_axis == 'torque_x':
                    val = math.degrees(roll_rad)
                elif self.effort_axis == 'torque_z':
                    val = math.degrees(yaw_rad)
                else:
                    val = math.degrees(pitch_rad)
            elif hasattr(msg, 'point'):
                if self.effort_axis == 'force_y':
                    val = msg.point.y
                else:
                    val = msg.point.x
            elif hasattr(msg, 'data'):
                val = msg.data
            with self.lock:
                self.state_history.append((now_sec, val))

    def effort_callback(self, msg):
        if self.is_recording:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            val = 0.0
            if hasattr(msg, 'force'):
                if self.effort_axis == 'force_x': val = msg.force.x
                elif self.effort_axis == 'force_y': val = msg.force.y
                elif self.effort_axis == 'force_z': val = msg.force.z
                elif self.effort_axis == 'torque_x': val = msg.torque.x
                elif self.effort_axis == 'torque_y': val = msg.torque.y
                elif self.effort_axis == 'torque_z': val = msg.torque.z
            with self.lock:
                self.effort_history.append((now_sec, val))

    def state_cb_att(self, msg):
        if self.is_recording:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            roll_rad, pitch_rad, yaw_rad = quaternion_to_euler(msg.orientation)
            with self.lock:
                if 'roll' in self.multi_state: self.multi_state['roll'].append((now_sec, math.degrees(roll_rad)))
                if 'pitch' in self.multi_state: self.multi_state['pitch'].append((now_sec, math.degrees(pitch_rad)))
                if 'yaw' in self.multi_state: self.multi_state['yaw'].append((now_sec, math.degrees(yaw_rad)))

    def state_cb_depth(self, msg):
        if self.is_recording:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            with self.lock:
                self.multi_state['depth'].append((now_sec, msg.data))

    def state_cb_pos(self, msg):
        if self.is_recording:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            with self.lock:
                self.multi_state['x'].append((now_sec, msg.point.x))
                self.multi_state['y'].append((now_sec, msg.point.y))

    def effort_cb_multi(self, msg):
        if self.is_recording:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            u_sq = msg.force.x**2 + msg.force.y**2 + msg.force.z**2 + msg.torque.x**2 + msg.torque.y**2 + msg.torque.z**2
            with self.lock:
                self.multi_effort.append((now_sec, u_sq))

    def load_tuner_config_from_yaml(self):
        """Load tuner parameters and mode definitions from cmaes_tuner_params.yaml."""
        modes_cfg = {}
        try:
            yaml_path = os.path.join(get_package_share_directory('controls'), 'params', 'cmaes_tuner_params.yaml')

            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                if 'cmaes_tuner' in data and 'ros__parameters' in data['cmaes_tuner']:
                    ros_params = data['cmaes_tuner']['ros__parameters']
                    modes_cfg = ros_params.get('modes', {})
                    # Load general algorithm defaults if CLI parameter wasn't explicitly provided
                    if self.get_parameter('popsize').value == 8 and 'popsize' in ros_params:
                        self.popsize = int(ros_params['popsize'])
                    if self.get_parameter('maxiter').value == 15 and 'maxiter' in ros_params:
                        self.maxiter = int(ros_params['maxiter'])
                    if self.get_parameter('sim_duration_sec').value == 10.0 and 'sim_duration_sec' in ros_params:
                        self.sim_duration_sec = float(ros_params['sim_duration_sec'])
                    if 'randomize_setpoints' in ros_params:
                        self.randomize_setpoints = bool(ros_params['randomize_setpoints'])
                    if 'weight_effort' in ros_params:
                        self.weight_effort = float(ros_params['weight_effort'])
                    if 'overshoot_threshold' in ros_params:
                        self.overshoot_threshold = float(ros_params['overshoot_threshold'])
                    if 'overshoot_floor_deg' in ros_params:
                        self.overshoot_floor_deg = float(ros_params['overshoot_floor_deg'])
                    if 'overshoot_floor_meters' in ros_params:
                        self.overshoot_floor_meters = float(ros_params['overshoot_floor_meters'])
                    if 'overshoot_penalty_base' in ros_params:
                        self.overshoot_penalty_base = float(ros_params['overshoot_penalty_base'])
        except Exception as e:
            self.get_logger().warn(f"Could not load cmaes_tuner_params.yaml: {e}")
        return modes_cfg

    def load_initial_guesses_from_yaml(self):
        """Automatically load initial PID guesses from Controller_params_sim.yaml."""
        try:
            yaml_path = os.path.join(get_package_share_directory('controls'), 'params', 'Controller_params_sim.yaml')
            
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as f:
                    params_data = yaml.safe_load(f)
                if getattr(self, 'is_multi_controller', False):
                    att_p = params_data.get('attitude_controller', {}).get('ros__parameters', {})
                    dep_p = params_data.get('depth_controller', {}).get('ros__parameters', {})
                    x_p = params_data.get('x_controller', {}).get('ros__parameters', {})
                    y_p = params_data.get('y_controller', {}).get('ros__parameters', {})
                    self.init_vector = [
                        float(att_p.get('P_ex', 6.0)), float(att_p.get('I_ex', 0.0)), float(att_p.get('P_wx', 1.0)),
                        float(att_p.get('P_ey', 6.0)), float(att_p.get('I_ey', 0.0)), float(att_p.get('P_wy', 1.0)),
                        float(att_p.get('P_ez', 6.0)), float(att_p.get('I_ez', 0.0)), float(att_p.get('P_wz', 1.0)),
                        float(dep_p.get('KP', 10.0)), float(dep_p.get('KI', 0.0)), float(dep_p.get('KD', 50.0)),
                        float(x_p.get('KP', 15.0)), float(x_p.get('KI', 0.5)), float(x_p.get('KD', 50.0)),
                        float(y_p.get('KP', 15.0)), float(y_p.get('KI', 0.5)), float(y_p.get('KD', 50.0)),
                    ]
                    self.get_logger().info("Auto-loaded 18-parameter baseline vector for FULL mode from YAML.")
                else:
                    ctrl_name = self.target_controller.lstrip('/')
                    if ctrl_name in params_data and 'ros__parameters' in params_data[ctrl_name]:
                        ros_params = params_data[ctrl_name]['ros__parameters']
                        if self.param_name_kp in ros_params:
                            self.init_kp = float(ros_params[self.param_name_kp])
                        if self.param_name_ki in ros_params:
                            self.init_ki = float(ros_params[self.param_name_ki])
                        if self.param_name_kd in ros_params:
                            self.init_kd = float(ros_params[self.param_name_kd])
                        self.get_logger().info(
                            f"Auto-loaded initial guesses from Controller_params_sim.yaml ({ctrl_name}): "
                            f"{self.param_name_kp}={self.init_kp}, {self.param_name_ki}={self.init_ki}, {self.param_name_kd}={self.init_kd}"
                        )
        except Exception as e:
            self.get_logger().warn(f"Could not auto-load from Controller_params_sim.yaml, using defaults: {e}")

    def set_auxiliary_controllers(self, enabled: bool):
        """Helper to keep auxiliary controllers enabled with YAML defaults during cascaded evaluation."""
        for ctrl in self.auxiliary_controllers:
            client = self.create_client(SetParameters, f"{ctrl}/set_parameters", callback_group=self.cb_group)
            if client.wait_for_service(timeout_sec=1.0):
                req = SetParameters.Request()
                param = Parameter()
                param.name = "enabled"
                param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled)
                req.parameters.append(param)
                client.call_async(req)

    def set_controller_params(self, param_dict):
        """Helper to inject candidate gains or enable/disable controller via ROS 2 parameter service."""
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Parameter service '{self.target_controller}/set_parameters' not available")
            return False

        req = SetParameters.Request()
        for name, val in param_dict.items():
            param = Parameter()
            param.name = name
            if isinstance(val, bool):
                param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=val)
            elif isinstance(val, int):
                param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=val)
            elif isinstance(val, float):
                param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=val)
            req.parameters.append(param)

        future = self.param_client.call_async(req)
        while not future.done() and rclpy.ok():
            time.sleep(0.005)  # Yield wall clock so MultiThreadedExecutor can resolve future
        
        if future.result() is not None:
            return all(res.successful for res in future.result().results)
        return False

    def send_param_request(self, controller_name, param_dict):
        """Helper to send parameter update requests to any specified controller."""
        client = self.create_client(SetParameters, f"{controller_name}/set_parameters", callback_group=self.cb_group)
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        req = SetParameters.Request()
        for name, val in param_dict.items():
            param = Parameter()
            param.name = str(name)
            if isinstance(val, bool):
                param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=val)
            elif isinstance(val, int):
                param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=val)
            elif isinstance(val, float):
                param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=val)
            req.parameters.append(param)
        future = client.call_async(req)
        while not future.done() and rclpy.ok():
            time.sleep(0.005)
        if future.result() is not None:
            return all(res.successful for res in future.result().results)
        return False

    def compute_cost(self, depth_data, effort_data):
        """
        REQUIREMENT 3: State-of-the-Art Cost Function (Reward Shaping)
        Computes ITAE + Effort Penalty (tau^2) + Discontinuous Critical Damping Overshoot Penalty.
        """
        if len(depth_data) < 5 or len(effort_data) < 5:
            self.get_logger().warn("Insufficient data collected during evaluation. Assigning max penalty.")
            return 10000.0, {'itae': 10000.0, 'effort': 0.0, 'overshoot': 0.0}

        # Unpack timestamps and values
        t_depth, y_depth = zip(*depth_data)
        t_effort, u_effort = zip(*effort_data)

        # Create a uniform simulated time grid for clean numerical integration
        t_start = max(t_depth[0], t_effort[0])
        t_end = min(t_depth[-1], t_effort[-1])
        if t_end <= t_start:
            return 10000.0, {'itae': 10000.0, 'effort': 0.0, 'overshoot': 0.0}

        num_points = int((t_end - t_start) * 50) + 10  # ~50 Hz grid
        t_grid = np.linspace(t_start, t_end, num=num_points)
        t_rel = t_grid - t_grid[0]

        # Interpolate state and effort onto uniform grid
        y_grid = np.interp(t_grid, t_depth, y_depth)
        u_grid = np.interp(t_grid, t_effort, u_effort)

        # Use current evaluation setpoint (synchronized per generation)
        eval_sp = getattr(self, 'current_eval_setpoint', self.target_setpoint)
        sp_norm = max(0.1, abs(eval_sp))

        # 1. ITAE (Integral Time Absolute Error), normalized by setpoint magnitude for scale invariance across random setpoints
        error = eval_sp - y_grid
        abs_error = np.abs(error)
        itae = float(np.trapz(t_rel * abs_error, t_rel)) / sp_norm

        # 2. Effort Penalty (tau^2), normalized by setpoint magnitude squared for scale invariance
        effort_penalty = float(self.weight_effort * np.trapz(u_grid**2, t_rel)) / (sp_norm ** 2)

        # 3. Overshoot Penalty (Discontinuous penalty for critical damping)
        # In marine robotics, overshooting dive target means crashing into seafloor
        y0 = y_grid[0]
        direction = np.sign(eval_sp - y0)
        if direction == 0:
            direction = 1.0

        # Calculate how far beyond target setpoint the AUV traveled
        overshoot_amt = np.maximum(0.0, direction * (y_grid - eval_sp))
        max_overshoot = float(np.max(overshoot_amt))

        abs_floor = self.overshoot_floor_deg if self.setpoint_topic == '/controls/quaternion_setpoint' else self.overshoot_floor_meters
        allowed_overshoot = max(abs_floor, self.overshoot_threshold * sp_norm)

        if max_overshoot > allowed_overshoot:
            # Massive discontinuous penalty + linear gradient to guide CMA-ES back to zero-overshoot
            excess_ratio = (max_overshoot - allowed_overshoot) / sp_norm
            overshoot_penalty = self.overshoot_penalty_base + 10000.0 * excess_ratio
        else:
            overshoot_penalty = 0.0

        total_cost = itae + effort_penalty + overshoot_penalty
        stats = {
            'itae': itae,
            'effort': effort_penalty,
            'overshoot': max_overshoot
        }
        return total_cost, stats

    def evaluate_candidate(self, kp, ki, kd):
        """
        REQUIREMENT 2: The Tuner Node Lifecycle
        Sequentially resets simulation, injects gains, enables controller, and records time-series.
        """
        # 1. Ensure all controllers are disabled before resetting pool
        for ctrl in ['/attitude_controller', '/depth_controller', '/x_controller', '/y_controller']:
            self.send_param_request(ctrl, {"enabled": False})
        time.sleep(0.05)

        # 2. Publish to /simulation/reset to teleport AUV to origin and randomize mass/drag
        msg = Bool()
        msg.data = True
        self.pub_reset.publish(msg)
        
        # Wait 0.5 simulated seconds for Unity physics to settle after teleportation
        reset_start = self.get_clock().now()
        while rclpy.ok():
            elapsed_reset = (self.get_clock().now() - reset_start).nanoseconds * 1e-9
            if elapsed_reset >= 0.5:
                break
            time.sleep(0.01)

        # 3. Enable auxiliary controllers to maintain stability (Cascaded Schedule)
        self.set_auxiliary_controllers(True)

        # 4. Use SetParameters client to inject candidate KP, KI, KD gains into active controller
        success = self.set_controller_params({
            self.param_name_kp: float(kp),
            self.param_name_ki: float(ki),
            self.param_name_kd: float(kd),
            "enabled": True
        })
        if not success:
            self.get_logger().warn("Failed to inject candidate parameters into controller")

        # Publish target setpoint (synchronized per generation)
        eval_sp = getattr(self, 'current_eval_setpoint', self.target_setpoint)
        if self.setpoint_topic == '/controls/quaternion_setpoint':
            if self.effort_axis == 'torque_x':
                euler_zyx = [0.0, 0.0, eval_sp]
            elif self.effort_axis == 'torque_z':
                euler_zyx = [eval_sp, 0.0, 0.0]
            else:
                euler_zyx = [0.0, eval_sp, 0.0]
            r = Rotation.from_euler('ZYX', euler_zyx, degrees=True)
            q_arr = r.as_quat()
            q_msg = Quaternion(x=float(q_arr[0]), y=float(q_arr[1]), z=float(q_arr[2]), w=float(q_arr[3]))
            self.pub_setpoint.publish(q_msg)
        else:
            setpoint_msg = Float64()
            setpoint_msg.data = float(eval_sp)
            self.pub_setpoint.publish(setpoint_msg)

        # 5. Spin and record state and control effort for fixed simulation duration
        with self.lock:
            self.state_history = []
            self.effort_history = []
            self.is_recording = True

        start_sim_time = self.get_clock().now()
        while rclpy.ok():
            elapsed_sim = (self.get_clock().now() - start_sim_time).nanoseconds * 1e-9
            if elapsed_sim >= self.sim_duration_sec:
                break
            time.sleep(0.01)  # Sleep wall clock while ROS 2 executor spins and receives /clock

        with self.lock:
            self.is_recording = False
            state_data = list(self.state_history)
            effort_data = list(self.effort_history)

        # 6. Disable all controllers after evaluation window
        for ctrl in ['/attitude_controller', '/depth_controller', '/x_controller', '/y_controller']:
            self.send_param_request(ctrl, {"enabled": False})

        return self.compute_cost(state_data, effort_data)

    def evaluate_candidate_multi(self, candidate):
        """Evaluate 18-parameter joint vector across all 4 controllers simultaneously."""
        for ctrl in ['/attitude_controller', '/depth_controller', '/x_controller', '/y_controller']:
            self.send_param_request(ctrl, {"enabled": False})
        time.sleep(0.05)

        msg = Bool()
        msg.data = True
        self.pub_reset.publish(msg)
        reset_start = self.get_clock().now()
        while rclpy.ok():
            if (self.get_clock().now() - reset_start).nanoseconds * 1e-9 >= 0.5:
                break
            time.sleep(0.01)

        att_params = {
            "P_ex": float(candidate[0]), "I_ex": float(candidate[1]), "P_wx": float(candidate[2]),
            "P_ey": float(candidate[3]), "I_ey": float(candidate[4]), "P_wy": float(candidate[5]),
            "P_ez": float(candidate[6]), "I_ez": float(candidate[7]), "P_wz": float(candidate[8]),
            "enabled": True
        }
        depth_params = {"KP": float(candidate[9]), "KI": float(candidate[10]), "KD": float(candidate[11]), "enabled": True}
        x_params = {"KP": float(candidate[12]), "KI": float(candidate[13]), "KD": float(candidate[14]), "enabled": True}
        y_params = {"KP": float(candidate[15]), "KI": float(candidate[16]), "KD": float(candidate[17]), "enabled": True}

        self.send_param_request('/attitude_controller', att_params)
        self.send_param_request('/depth_controller', depth_params)
        self.send_param_request('/x_controller', x_params)
        self.send_param_request('/y_controller', y_params)

        # Publish diagonal 3D maneuver setpoints (using scipy Rotation for any 3D angle direction)
        eval_ang = getattr(self, 'current_eval_angular_setpoint', getattr(self, 'default_angular_setpoint', [10.0, 15.0, 30.0]))
        if isinstance(eval_ang, (list, tuple, np.ndarray)) and len(eval_ang) >= 3:
            eval_roll, eval_pitch, eval_yaw = float(eval_ang[0]), float(eval_ang[1]), float(eval_ang[2])
        else:
            eval_roll, eval_pitch, eval_yaw = 0.0, 0.0, float(eval_ang)
            
        r = Rotation.from_euler('ZYX', [eval_yaw, eval_pitch, eval_roll], degrees=True)
        q_arr = r.as_quat()
        q_msg = Quaternion(x=float(q_arr[0]), y=float(q_arr[1]), z=float(q_arr[2]), w=float(q_arr[3]))
        self.pub_sp_quat.publish(q_msg)
        eval_trans = getattr(self, 'current_eval_setpoint', getattr(self, 'default_setpoint', [5.0, 5.0, 2.0]))
        if isinstance(eval_trans, (list, tuple, np.ndarray)) and len(eval_trans) >= 3:
            eval_x, eval_y, eval_depth = float(eval_trans[0]), float(eval_trans[1]), float(eval_trans[2])
        else:
            eval_x, eval_y, eval_depth = float(eval_trans), float(eval_trans), 2.0

        self.pub_sp_depth.publish(Float64(data=eval_depth))
        self.pub_sp_x.publish(Float64(data=eval_x))
        self.pub_sp_y.publish(Float64(data=eval_y))

        with self.lock:
            self.multi_state = {'roll': [], 'pitch': [], 'yaw': [], 'depth': [], 'x': [], 'y': []}
            self.multi_effort = []
            self.is_recording = True

        start_sim = self.get_clock().now()
        while rclpy.ok():
            if (self.get_clock().now() - start_sim).nanoseconds * 1e-9 >= self.sim_duration_sec:
                break
            time.sleep(0.01)

        with self.lock:
            self.is_recording = False
            multi_state_data = {k: list(v) for k, v in self.multi_state.items()}
            multi_eff_data = list(self.multi_effort)

        for ctrl in ['/attitude_controller', '/depth_controller', '/x_controller', '/y_controller']:
            self.send_param_request(ctrl, {"enabled": False})

        return self.compute_cost_multi(multi_state_data, multi_eff_data)

    def compute_cost_multi(self, multi_state_data, multi_eff_data):
        total_itae = 0.0
        total_os = 0.0
        eval_ang = getattr(self, 'current_eval_angular_setpoint', getattr(self, 'default_angular_setpoint', [10.0, 15.0, 30.0]))
        if isinstance(eval_ang, (list, tuple, np.ndarray)) and len(eval_ang) >= 3:
            eval_roll, eval_pitch, eval_yaw = float(eval_ang[0]), float(eval_ang[1]), float(eval_ang[2])
        else:
            eval_roll, eval_pitch, eval_yaw = 0.0, 0.0, float(eval_ang)

        eval_trans = getattr(self, 'current_eval_setpoint', getattr(self, 'default_setpoint', [5.0, 5.0, 2.0]))
        if isinstance(eval_trans, (list, tuple, np.ndarray)) and len(eval_trans) >= 3:
            eval_x, eval_y, eval_depth = float(eval_trans[0]), float(eval_trans[1]), float(eval_trans[2])
        else:
            eval_x, eval_y, eval_depth = float(eval_trans), float(eval_trans), 2.0

        sps = {
            'roll': eval_roll,
            'pitch': eval_pitch,
            'yaw': eval_yaw,
            'depth': eval_depth,
            'x': eval_x,
            'y': eval_y
        }

        max_os_pct = 0.0
        for axis, sp in sps.items():
            hist = multi_state_data.get(axis, [])
            if len(hist) < 5:
                return 100000.0, {'itae': 100000.0, 'effort': 0.0, 'overshoot': 0.0}
            t_vals = np.array([pt[0] for pt in hist])
            y_vals = np.array([pt[1] for pt in hist])
            t_grid = np.linspace(t_vals[0], t_vals[-1], int(self.sim_duration_sec * 50))
            t_rel = t_grid - t_grid[0]
            y_grid = np.interp(t_grid, t_vals, y_vals)
            
            sp_norm = max(0.1, abs(sp))
            error = sp - y_grid
            itae = float(np.trapz(t_rel * np.abs(error), t_rel)) / sp_norm
            total_itae += itae

            y0 = y_grid[0]
            direction = np.sign(sp - y0)
            if direction == 0: direction = 1.0
            overshoot_amt = np.maximum(0.0, direction * (y_grid - sp))
            abs_floor = self.overshoot_floor_deg if axis in ['roll', 'pitch', 'yaw'] else self.overshoot_floor_meters
            allowed_overshoot = max(abs_floor, self.overshoot_threshold * sp_norm)
            max_os_val = float(np.max(overshoot_amt))
            os_pct = (max_os_val / sp_norm) * 100.0
            if os_pct > max_os_pct:
                max_os_pct = os_pct
            if max_os_val > allowed_overshoot:
                excess_ratio = (max_os_val - allowed_overshoot) / sp_norm
                total_os += (self.overshoot_penalty_base + 10000.0 * excess_ratio)

        eff_pen = 0.0
        if len(multi_eff_data) >= 5:
            t_eff = np.array([pt[0] for pt in multi_eff_data])
            u_sq = np.array([pt[1] for pt in multi_eff_data])
            t_grid = np.linspace(t_eff[0], t_eff[-1], int(self.sim_duration_sec * 50))
            t_rel = t_grid - t_grid[0]
            u_grid = np.interp(t_grid, t_eff, u_sq)
            avg_sp_norm_sq = float(np.mean([max(0.1, abs(sp))**2 for sp in sps.values()]))
            eff_pen = float(self.weight_effort * np.trapz(u_grid, t_rel)) / avg_sp_norm_sq

        total_cost = total_itae + eff_pen + total_os
        return total_cost, {'itae': total_itae, 'effort': eff_pen, 'overshoot': max_os_pct, 'overshoot_penalty': total_os}

    def run_optimization(self):
        """Main CMA-ES optimization loop."""
        self.get_logger().info("========================================================")
        self.get_logger().info("STARTING CMA-ES SITL AUTOMATED PID TUNING PIPELINE")
        self.get_logger().info(f"Target Controller: {self.target_controller} | Setpoint: {self.target_setpoint}m/deg")
        self.get_logger().info(f"Evaluation Window: {self.sim_duration_sec} sim seconds | Pop Size: {self.popsize}")
        self.get_logger().info("========================================================")

        # Wait for simulation clock (/clock) to be active
        while self.get_clock().now().nanoseconds == 0 and rclpy.ok():
            self.get_logger().warn("Waiting for /clock from Unity simulation...", throttle_duration_sec=2.0)
            time.sleep(0.1)

        self.get_logger().info("Simulation clock detected. Disabling all controllers before starting optimization...")
        for ctrl in ['/attitude_controller', '/depth_controller', '/x_controller', '/y_controller']:
            self.send_param_request(ctrl, {"enabled": False})
        time.sleep(0.1)

        if getattr(self, 'is_multi_controller', False):
            x0 = getattr(self, 'init_vector', [6.0, 0.0, 1.0, 6.0, 0.0, 1.0, 6.0, 0.0, 1.0, 10.0, 0.0, 50.0, 15.0, 0.5, 50.0, 15.0, 0.5, 50.0])
        else:
            x0 = [self.init_kp, self.init_ki, self.init_kd]

        opts = {
            'bounds': [0.0, np.inf],  # All PID gains must be strictly non-negative
            'popsize': self.popsize,
            'maxiter': self.maxiter,
            'verb_disp': 0,
        }

        es = cma.CMAEvolutionStrategy(x0, self.init_sigma, opts)
        
        generation = 0
        best_overall_cost = float('inf')
        best_overall_gains = None

        while not es.stop() and rclpy.ok():
            generation += 1
            if getattr(self, 'randomize_setpoints', False) and hasattr(self, 'setpoint_range'):
                smin, smax = self.setpoint_range[0], self.setpoint_range[1]
                if isinstance(smin, (list, tuple)) and isinstance(smax, (list, tuple)) and len(smin) >= 3 and len(smax) >= 3:
                    self.current_eval_setpoint = [
                        float(np.random.uniform(smin[0], smax[0])),
                        float(np.random.uniform(smin[1], smax[1])),
                        float(np.random.uniform(smin[2], smax[2]))
                    ]
                else:
                    self.current_eval_setpoint = float(np.random.uniform(float(smin), float(smax)))

                if hasattr(self, 'angular_setpoint_range'):
                    amin, amax = self.angular_setpoint_range[0], self.angular_setpoint_range[1]
                    if isinstance(amin, (list, tuple)) and isinstance(amax, (list, tuple)) and len(amin) >= 3 and len(amax) >= 3:
                        self.current_eval_angular_setpoint = [
                            float(np.random.uniform(amin[0], amax[0])),
                            float(np.random.uniform(amin[1], amax[1])),
                            float(np.random.uniform(amin[2], amax[2]))
                        ]
                    else:
                        self.current_eval_angular_setpoint = float(np.random.uniform(float(amin), float(amax)))
                else:
                    self.current_eval_angular_setpoint = getattr(self, 'default_angular_setpoint', [10.0, 15.0, 30.0])
                
                if getattr(self, 'is_multi_controller', False):
                    trans_str = f"[{self.current_eval_setpoint[0]:.2f}m, {self.current_eval_setpoint[1]:.2f}m, {self.current_eval_setpoint[2]:.2f}m]" if isinstance(self.current_eval_setpoint, (list, tuple)) else f"{self.current_eval_setpoint:.2f}m"
                    ang_str = f"[{self.current_eval_angular_setpoint[0]:.1f}°, {self.current_eval_angular_setpoint[1]:.1f}°, {self.current_eval_angular_setpoint[2]:.1f}°]" if isinstance(self.current_eval_angular_setpoint, (list, tuple)) else f"{self.current_eval_angular_setpoint:.1f}°"
                    self.get_logger().info(f"\n--- GENERATION {generation}/{self.maxiter} | Synchronized Setpoints -> Translation [X,Y,Depth]: {trans_str}, Rotation [R,P,Y]: {ang_str} ---")
                else:
                    self.get_logger().info(f"\n--- GENERATION {generation}/{self.maxiter} | Synchronized Setpoint: {self.current_eval_setpoint:.2f} (Range: {self.setpoint_range}) ---")
            else:
                self.current_eval_setpoint = self.target_setpoint
                self.current_eval_angular_setpoint = getattr(self, 'default_angular_setpoint', [10.0, 15.0, 30.0])
                self.get_logger().info(f"\n--- GENERATION {generation}/{self.maxiter} ---")

            solutions = es.ask()
            costs = []

            for idx, candidate in enumerate(solutions):
                if not rclpy.ok():
                    break

                if getattr(self, 'is_multi_controller', False):
                    att_str = f"Att(Roll P={candidate[0]:.2f}, I={candidate[1]:.3f}, D={candidate[2]:.2f} | Pitch P={candidate[3]:.2f}, I={candidate[4]:.3f}, D={candidate[5]:.2f} | Yaw P={candidate[6]:.2f}, I={candidate[7]:.3f}, D={candidate[8]:.2f})"
                    dep_str = f"Depth(P={candidate[9]:.2f}, I={candidate[10]:.3f}, D={candidate[11]:.2f})"
                    x_str = f"Surge(P={candidate[12]:.2f}, I={candidate[13]:.3f}, D={candidate[14]:.2f})"
                    y_str = f"Sway(P={candidate[15]:.2f}, I={candidate[16]:.3f}, D={candidate[17]:.2f})"
                    self.get_logger().info(f"[Gen {generation} | Candidate {idx+1}/{len(solutions)}] Testing 18-parameter joint vector:")
                    self.get_logger().info(f"    -> {att_str}")
                    self.get_logger().info(f"    -> {dep_str} | {x_str} | {y_str}")
                    cost, stats = self.evaluate_candidate_multi(candidate)
                    costs.append(cost)
                    self.get_logger().info(
                        f"  -> Cost: {cost:.1f} | ITAE: {stats['itae']:.1f} | Effort: {stats['effort']:.2f} | Max Overshoot: {stats['overshoot']:.2f}%"
                    )
                    if cost < best_overall_cost:
                        best_overall_cost = cost
                        best_overall_gains = candidate
                        self.get_logger().info(f"  NEW BEST JOINT GAINS FOUND: Cost: {best_overall_cost:.1f}")
                else:
                    kp, ki, kd = candidate[0], candidate[1], candidate[2]
                    self.get_logger().info(f"[Gen {generation} | Candidate {idx+1}/{len(solutions)}] Testing KP={kp:.2f}, KI={ki:.2f}, KD={kd:.2f}...")

                    cost, stats = self.evaluate_candidate(kp, ki, kd)
                    costs.append(cost)

                    unit_str = "°" if self.setpoint_topic == '/controls/quaternion_setpoint' else "m"
                    self.get_logger().info(
                        f"  -> Cost: {cost:.1f} | ITAE: {stats['itae']:.1f} | Effort: {stats['effort']:.2f} | Max Overshoot: {stats['overshoot']:.4f}{unit_str}"
                    )

                    if cost < best_overall_cost:
                        best_overall_cost = cost
                        best_overall_gains = (kp, ki, kd)
                        self.get_logger().info(f"  NEW BEST GAINS FOUND: Cost: {best_overall_cost:.1f} (KP={kp:.2f}, KI={ki:.2f}, KD={kd:.2f})")

            if not rclpy.ok():
                break

            es.tell(solutions, costs)
            es.disp()

        if best_overall_gains is not None and rclpy.ok():
            self.get_logger().info("\n========================================================")
            self.get_logger().info("CMA-ES AUTOMATED TUNING COMPLETE")
            self.get_logger().info(f"Best Overall Cost: {best_overall_cost:.2f}")
            if getattr(self, 'is_multi_controller', False):
                self.get_logger().info("Winning 18-Parameter Joint Baseline Vector:")
                att_w = f"P_ex={best_overall_gains[0]:.2f}, I_ex={best_overall_gains[1]:.4f}, P_wx={best_overall_gains[2]:.2f}, P_ey={best_overall_gains[3]:.2f}, I_ey={best_overall_gains[4]:.4f}, P_wy={best_overall_gains[5]:.2f}, P_ez={best_overall_gains[6]:.2f}, I_ez={best_overall_gains[7]:.4f}, P_wz={best_overall_gains[8]:.2f}"
                dep_w = f"KP={best_overall_gains[9]:.2f}, KI={best_overall_gains[10]:.4f}, KD={best_overall_gains[11]:.2f}"
                x_w = f"KP={best_overall_gains[12]:.2f}, KI={best_overall_gains[13]:.4f}, KD={best_overall_gains[14]:.2f}"
                y_w = f"KP={best_overall_gains[15]:.2f}, KI={best_overall_gains[16]:.4f}, KD={best_overall_gains[17]:.2f}"
                self.get_logger().info(f"  /attitude_controller: {att_w}")
                self.get_logger().info(f"  /depth_controller:    {dep_w}")
                self.get_logger().info(f"  /x_controller:        {x_w}")
                self.get_logger().info(f"  /y_controller:        {y_w}")
                
                self.send_param_request('/attitude_controller', {"P_ex": float(best_overall_gains[0]), "I_ex": float(best_overall_gains[1]), "P_wx": float(best_overall_gains[2]), "P_ey": float(best_overall_gains[3]), "I_ey": float(best_overall_gains[4]), "P_wy": float(best_overall_gains[5]), "P_ez": float(best_overall_gains[6]), "I_ez": float(best_overall_gains[7]), "P_wz": float(best_overall_gains[8]), "enabled": True})
                self.send_param_request('/depth_controller', {"KP": float(best_overall_gains[9]), "KI": float(best_overall_gains[10]), "KD": float(best_overall_gains[11]), "enabled": True})
                self.send_param_request('/x_controller', {"KP": float(best_overall_gains[12]), "KI": float(best_overall_gains[13]), "KD": float(best_overall_gains[14]), "enabled": True})
                self.send_param_request('/y_controller', {"KP": float(best_overall_gains[15]), "KI": float(best_overall_gains[16]), "KD": float(best_overall_gains[17]), "enabled": True})
                self.get_logger().info("Applied joint best gains to all controllers. AUV ready for mission")
            else:
                self.get_logger().info(
                    f"Best Gains -> KP: {best_overall_gains[0]:.4f}, KI: {best_overall_gains[1]:.4f}, KD: {best_overall_gains[2]:.4f}"
                )
                self.set_controller_params({
                    self.param_name_kp: float(best_overall_gains[0]),
                    self.param_name_ki: float(best_overall_gains[1]),
                    self.param_name_kd: float(best_overall_gains[2]),
                    "enabled": True
                })
                self.get_logger().info("Applied best gains to controller. AUV ready for mission")
            self.get_logger().info("========================================================")


def main():
    parser = argparse.ArgumentParser(description="CMA-ES Automated PID Tuner for AUV")
    parser.add_argument('--tune', type=str, default=None, choices=['attitude', 'depth', 'planar', 'surge', 'sway', 'pitch', 'roll', 'yaw', 'heave', 'full', 'joint', 'all'],
                        help="Cascaded tuning schedule mode: attitude/pitch/roll/yaw (Phase 1), depth/heave (Phase 2), planar/surge/sway (Phase 3), full/joint/all (Phase 4)")
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = CMAESTunerNode(tune_mode=args.tune)
    
    # Use MultiThreadedExecutor so callbacks and parameter services process concurrently with tuner thread
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Tuner stopped by user.")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
