#!/usr/bin/env python3
"""
Teleop Converter Node

Converts gamepad (Joy) inputs into Wrench commands (Propulsion Mode) or 
Pose Setpoints (Controls Mode) using SDL2 standard game_controller_node mappings.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Wrench, PoseStamped
from std_msgs.msg import Float64
from auv_msgs.msg import AttitudeReference

import tf_transformations
from controls.utils import normalize_angle


class TeleopConverter(Node):
    """Converts Joy inputs to Wrench/Setpoint outputs for manual AUV control."""

    def __init__(self):
        super().__init__('teleop_converter')
        
        # =================================================================
        # Parameters - Scaling (Propulsion Mode)
        # =================================================================
        self.declare_parameter('surge_scale', 20.0)   # Newtons
        self.declare_parameter('sway_scale', 20.0)    # Newtons
        self.declare_parameter('heave_scale', 40.0)   # Newtons
        self.declare_parameter('yaw_scale', 5.0)      # Nm
        self.declare_parameter('roll_scale', 2.0)     # Nm
        self.declare_parameter('pitch_scale', 2.0)    # Nm
        
        # =================================================================
        # Parameters - Max Offsets (Controls Mode)
        # =================================================================
        self.declare_parameter('surge_max_offset', 0.5)   # meters
        self.declare_parameter('sway_max_offset', 0.5)    # meters
        self.declare_parameter('depth_max_offset', 0.2)   # meters
        self.declare_parameter('yaw_max_offset', 0.5)     # rad
        self.declare_parameter('roll_max_offset', 0.5)    # rad
        self.declare_parameter('pitch_max_offset', 0.5)   # rad

        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('timeout_sec', 0.5)
        
        # =================================================================
        # Parameters - Axis/Button Indices (SDL2 game_controller_node)
        # =================================================================
        self.declare_parameter('axis_left_x', 0)
        self.declare_parameter('axis_left_y', 1)
        self.declare_parameter('axis_right_x', 2)
        self.declare_parameter('axis_right_y', 3)
        self.declare_parameter('axis_lt', 4)
        self.declare_parameter('axis_rt', 5)
        
        self.declare_parameter('btn_controls_mode', 9)    # LB
        self.declare_parameter('btn_propulsion_mode', 10) # RB
        self.declare_parameter('btn_dpad_up', 11)
        self.declare_parameter('btn_dpad_down', 12)
        self.declare_parameter('btn_dpad_left', 13)
        self.declare_parameter('btn_dpad_right', 14)
        
        # Load parameters
        self.surge_scale = self.get_parameter('surge_scale').value
        self.sway_scale = self.get_parameter('sway_scale').value
        self.heave_scale = self.get_parameter('heave_scale').value
        self.yaw_scale = self.get_parameter('yaw_scale').value
        self.roll_scale = self.get_parameter('roll_scale').value
        self.pitch_scale = self.get_parameter('pitch_scale').value
        
        self.surge_max_offset = self.get_parameter('surge_max_offset').value
        self.sway_max_offset = self.get_parameter('sway_max_offset').value
        self.depth_max_offset = self.get_parameter('depth_max_offset').value
        self.yaw_max_offset = self.get_parameter('yaw_max_offset').value
        self.roll_max_offset = self.get_parameter('roll_max_offset').value
        self.pitch_max_offset = self.get_parameter('pitch_max_offset').value
        
        self.deadzone = self.get_parameter('deadzone').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        
        self.axis_left_x = self.get_parameter('axis_left_x').value
        self.axis_left_y = self.get_parameter('axis_left_y').value
        self.axis_right_x = self.get_parameter('axis_right_x').value
        self.axis_right_y = self.get_parameter('axis_right_y').value
        self.axis_lt = self.get_parameter('axis_lt').value
        self.axis_rt = self.get_parameter('axis_rt').value
        
        self.btn_controls_mode = self.get_parameter('btn_controls_mode').value
        self.btn_propulsion_mode = self.get_parameter('btn_propulsion_mode').value
        self.btn_dpad_up = self.get_parameter('btn_dpad_up').value
        self.btn_dpad_down = self.get_parameter('btn_dpad_down').value
        self.btn_dpad_left = self.get_parameter('btn_dpad_left').value
        self.btn_dpad_right = self.get_parameter('btn_dpad_right').value
        
        # =================================================================
        # State
        # =================================================================
        self.last_joy_time = None
        self.dt = 0.05 # 20Hz loop
        
        # State estimation data
        self.current_pose = None
        self.setpoints_initialized = False
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        
        # Target setpoints (Controls Mode)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_depth = 0.0
        self.target_yaw = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        
        self.xy_active = False
        self.z_active = False
        self.yaw_active = False
        self.pitch_active = False
        self.roll_active = False
        
        # Parameter Clients for toggling PID controllers and Superimposer
        self.controller_nodes = ['attitude_controller', 'depth_controller', 'x_controller', 'y_controller', 'superimposer']
        self.param_clients = {}
        for node_name in self.controller_nodes:
            client = self.create_client(SetParameters, f'/{node_name}/set_parameters')
            self.param_clients[node_name] = client
            
        # Output wrench (Propulsion Mode)
        self.current_wrench = Wrench()
        
        self.controls_mode_active = False
        self.propulsion_mode_active = False
        
        # =================================================================
        # ROS Interfaces
        # =================================================================
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/state/pose', self.pose_callback, qos_profile_sensor_data)
        
        # Publishers (Propulsion)
        self.wrench_pub = self.create_publisher(
            Wrench, '/controls/total_effort', 10)
            
        # Publishers (Controls)
        self.x_sp_pub = self.create_publisher(Float64, '/controls/x_setpoint', 10)
        self.y_sp_pub = self.create_publisher(Float64, '/controls/y_setpoint', 10)
        self.depth_sp_pub = self.create_publisher(Float64, '/controls/depth_setpoint', 10)
        self.attitude_sp_pub = self.create_publisher(AttitudeReference, '/controls/attitude_reference', 10)
        
        # Timer
        self.timer = self.create_timer(self.dt, self.publish_loop)
        
        self.get_logger().info('Teleop Converter initialized (Dual Mode)')

    def pose_callback(self, msg: PoseStamped):
        """Keep track of the AUV's current global position and yaw."""
        self.current_pose = msg
        q = msg.pose.orientation
        # Use intrinsic Z-Y-X axes (rzyx) which gives us (yaw, pitch, roll)
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w], axes='rzyx')
        self.current_yaw = euler[0]
        self.current_pitch = euler[1]
        self.current_roll = euler[2]
        
        if not self.setpoints_initialized:
            self.target_x = self.current_pose.pose.position.x
            self.target_y = self.current_pose.pose.position.y
            self.target_depth = -self.current_pose.pose.position.z
            self.target_roll = self.current_roll
            self.target_pitch = self.current_pitch
            self.target_yaw = self.current_yaw
            self.setpoints_initialized = True
            self.get_logger().info(f'Initialized Setpoints -> X:{self.target_x:.2f}, Y:{self.target_y:.2f}, Z:{self.target_depth:.2f}, Yaw:{self.target_yaw:.2f}, Roll:{self.target_roll:.2f}, Pitch:{self.target_pitch:.2f}')

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def process_trigger(self, raw_val: float) -> float:
        """
        Convert trigger axis to 0.0 - 1.0 range.
        Based on hardware: unpressed is 0.0, fully pressed is -1.0.
        """
        return -raw_val

    def joy_callback(self, msg: Joy):
        """Process gamepad input."""
        self.last_joy_time = self.get_clock().now()
        
        # =================================================================
        # Mode Evaluation
        # =================================================================
        lb_held = False
        if len(msg.buttons) > self.btn_controls_mode:
            lb_held = bool(msg.buttons[self.btn_controls_mode])
            
        rb_held = False
        if len(msg.buttons) > self.btn_propulsion_mode:
            rb_held = bool(msg.buttons[self.btn_propulsion_mode])
            
        just_entered_controls = lb_held and not self.controls_mode_active
        just_exited_controls = not lb_held and self.controls_mode_active
        just_entered_propulsion = rb_held and not self.propulsion_mode_active
        just_exited_propulsion = not rb_held and self.propulsion_mode_active
        # Controls Mode Toggles
        if just_entered_controls:
            self.get_logger().info('Entered Controls Mode. Re-enabling PID controllers (resuming from last setpoint).')
            self.set_controllers_enabled(True)
            self.xy_active = False
            self.z_active = False
            self.yaw_active = False
            self.pitch_active = False
            self.roll_active = False
            if self.current_pose is None:
                self.get_logger().warn('Controls Mode requested but no /state/pose received yet')
                
        if just_exited_controls:
            if self.current_pose is not None:
                self.get_logger().info('Exited Controls Mode. Halting actively driven axes at current pose.')
                
                if self.xy_active:
                    self.target_x = self.current_pose.pose.position.x
                    self.target_y = self.current_pose.pose.position.y
                if self.z_active:
                    self.target_depth = -self.current_pose.pose.position.z
                if self.roll_active:
                    self.target_roll = self.current_roll
                if self.pitch_active:
                    self.target_pitch = self.current_pitch
                if self.yaw_active:
                    self.target_yaw = self.current_yaw
                
                # Publish the halting setpoints once directly
                msg_x = Float64()
                msg_x.data = self.target_x
                self.x_sp_pub.publish(msg_x)
                
                msg_y = Float64()
                msg_y.data = self.target_y
                self.y_sp_pub.publish(msg_y)
                
                msg_depth = Float64()
                msg_depth.data = self.target_depth
                self.depth_sp_pub.publish(msg_depth)
                
                msg_att = AttitudeReference()
                q_arr = tf_transformations.quaternion_from_euler(self.target_yaw, self.target_pitch, self.target_roll, axes='rzyx')
                msg_att.orientation.x = q_arr[0]
                msg_att.orientation.y = q_arr[1]
                msg_att.orientation.z = q_arr[2]
                msg_att.orientation.w = q_arr[3]
                msg_att.angular_velocity.x = 0.0
                msg_att.angular_velocity.y = 0.0
                msg_att.angular_velocity.z = 0.0
                self.attitude_sp_pub.publish(msg_att)
        if just_entered_propulsion:
            self.get_logger().info('Entered Propulsion Mode. Disabling PID controllers to prevent fighting.')
            self.set_controllers_enabled(False)
            
        if just_exited_propulsion:
            self.set_controllers_enabled(True)
            if self.current_pose is not None:
                self.target_x = self.current_pose.pose.position.x
                self.target_y = self.current_pose.pose.position.y
                self.target_depth = -self.current_pose.pose.position.z
                self.target_roll = self.current_roll
                self.target_pitch = self.current_pitch
                self.target_yaw = self.current_yaw
                self.get_logger().info(f'Exited Propulsion Mode. Re-enabling PID controllers and latching new pose -> X:{self.target_x:.2f}, Y:{self.target_y:.2f}, Z:{self.target_depth:.2f}, Yaw:{self.target_yaw:.2f}, Roll:{self.target_roll:.2f}, Pitch:{self.target_pitch:.2f}')
            else:
                self.get_logger().info('Exited Propulsion Mode. Re-enabling PID controllers.')
                
        self.controls_mode_active = lb_held
        self.propulsion_mode_active = rb_held
        
        # =================================================================
        # Extract axis values
        # =================================================================
        surge_axis = 0.0
        sway_axis = 0.0
        yaw_axis = 0.0
        heave_axis = 0.0
        roll_axis = 0.0
        pitch_axis = 0.0
        
        if len(msg.axes) > self.axis_left_y:
            surge_axis = self.apply_deadzone(msg.axes[self.axis_left_y])
        if len(msg.axes) > self.axis_left_x:
            sway_axis = self.apply_deadzone(msg.axes[self.axis_left_x])
            
        if len(msg.axes) > self.axis_right_x:
            yaw_axis = self.apply_deadzone(msg.axes[self.axis_right_x])
        
        if len(msg.axes) > max(self.axis_lt, self.axis_rt):
            lt = self.process_trigger(msg.axes[self.axis_lt])
            rt = self.process_trigger(msg.axes[self.axis_rt])
            heave_axis = rt - lt  # Positive = up, Negative = down
        
        if len(msg.buttons) > max(self.btn_dpad_up, self.btn_dpad_down, self.btn_dpad_left, self.btn_dpad_right):
            # Cancel out if both opposing buttons are pressed
            roll_axis = float(msg.buttons[self.btn_dpad_right]) - float(msg.buttons[self.btn_dpad_left])
            pitch_axis = float(msg.buttons[self.btn_dpad_up]) - float(msg.buttons[self.btn_dpad_down])

        # =================================================================
        # Propulsion Mode Logic (Raw Wrench)
        # =================================================================
        if self.propulsion_mode_active:
            wrench = Wrench()
            wrench.force.x = surge_axis * self.surge_scale
            wrench.force.y = sway_axis * self.sway_scale
            wrench.force.z = heave_axis * self.heave_scale
            wrench.torque.x = roll_axis * self.roll_scale
            wrench.torque.y = pitch_axis * self.pitch_scale
            wrench.torque.z = yaw_axis * self.yaw_scale
            self.current_wrench = wrench
            
            # Continuously update the target setpoints to perfectly match the current pose 
            # so the PID loops don't wind up, and we have a smooth transition back to Controls Mode.
            if self.current_pose is not None:
                self.target_x = self.current_pose.pose.position.x
                self.target_y = self.current_pose.pose.position.y
                self.target_depth = -self.current_pose.pose.position.z
                self.target_yaw = self.current_yaw
                self.target_roll = self.current_roll
                self.target_pitch = self.current_pitch
        else:
            self.current_wrench = Wrench()

        # =================================================================
        # Controls Mode Logic (Setpoint Offset)
        # =================================================================
        if self.controls_mode_active and self.current_pose is not None:
            # X/Y Plane
            if surge_axis != 0.0 or sway_axis != 0.0:
                dx_body = surge_axis * self.surge_max_offset
                dy_body = sway_axis * self.sway_max_offset
                cy = math.cos(self.current_yaw)
                sy = math.sin(self.current_yaw)
                self.target_x = self.current_pose.pose.position.x + (dx_body * cy - dy_body * sy)
                self.target_y = self.current_pose.pose.position.y + (dx_body * sy + dy_body * cy)
                self.xy_active = True
            elif self.xy_active:
                # Stick just released: Latch to current pose to hold position
                self.target_x = self.current_pose.pose.position.x
                self.target_y = self.current_pose.pose.position.y
                self.xy_active = False
                
            # Z (Heave)
            if heave_axis != 0.0:
                self.target_depth = -self.current_pose.pose.position.z - (heave_axis * self.depth_max_offset)
                self.z_active = True
            elif self.z_active:
                self.target_depth = -self.current_pose.pose.position.z
                self.z_active = False
                
            # Yaw
            if yaw_axis != 0.0:
                self.target_yaw = normalize_angle(self.current_yaw + yaw_axis * self.yaw_max_offset)
                self.yaw_active = True
            elif self.yaw_active:
                self.target_yaw = self.current_yaw
                self.yaw_active = False
                
            # Pitch
            if pitch_axis != 0.0:
                self.target_pitch = self.current_pitch + pitch_axis * self.pitch_max_offset
                self.pitch_active = True
            elif self.pitch_active:
                self.target_pitch = self.current_pitch
                self.pitch_active = False
                
            # Roll
            if roll_axis != 0.0:
                self.target_roll = self.current_roll + roll_axis * self.roll_max_offset
                self.roll_active = True
            elif self.roll_active:
                self.target_roll = self.current_roll
                self.roll_active = False

    def set_controllers_enabled(self, enabled: bool):
        """Asynchronously toggles the enabled parameter on all active PID controllers."""
        for node_name, client in self.param_clients.items():
            if not client.service_is_ready():
                self.get_logger().debug(f'Service /{node_name}/set_parameters not ready')
                continue
            
            req = SetParameters.Request()
            param = Parameter()
            param.name = "enabled"
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = enabled
            req.parameters.append(param)
            
            client.call_async(req)

    def publish_loop(self):
        """Publish outputs at fixed rate, with timeout check."""
        now = self.get_clock().now()
        
        joy_stale = True
        if self.last_joy_time is not None:
            elapsed = (now - self.last_joy_time).nanoseconds / 1e9
            joy_stale = elapsed > self.timeout_sec
            
        if joy_stale:
            self.current_wrench = Wrench()
            self.controls_mode_active = False
            self.propulsion_mode_active = False
        
        # Only publish Wrench if we are actively in propulsion mode
        if self.propulsion_mode_active:
            self.wrench_pub.publish(self.current_wrench)
        
        # Publish Setpoints if Controls mode OR Propulsion mode is active
        if (self.controls_mode_active or self.propulsion_mode_active) and self.current_pose is not None:
            msg_x = Float64()
            msg_x.data = self.target_x
            self.x_sp_pub.publish(msg_x)
            
            msg_y = Float64()
            msg_y.data = self.target_y
            self.y_sp_pub.publish(msg_y)
            
            msg_depth = Float64()
            msg_depth.data = self.target_depth
            self.depth_sp_pub.publish(msg_depth)
            
            msg_att = AttitudeReference()
            q_arr = tf_transformations.quaternion_from_euler(self.target_yaw, self.target_pitch, self.target_roll, axes='rzyx')
            msg_att.orientation.x = q_arr[0]
            msg_att.orientation.y = q_arr[1]
            msg_att.orientation.z = q_arr[2]
            msg_att.orientation.w = q_arr[3]
            
            # Position holding angular velocities
            msg_att.angular_velocity.x = 0.0
            msg_att.angular_velocity.y = 0.0
            msg_att.angular_velocity.z = 0.0
            
            self.attitude_sp_pub.publish(msg_att)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
