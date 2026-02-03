"""
Teleop Converter Node

Converts gamepad (Joy) and Foxglove/keyboard (Twist) inputs into Wrench commands
for manual AUV control. Implements deadman switch and safety timeout.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Wrench


class TeleopConverter(Node):
    """Converts Joy/Twist inputs to Wrench output for thruster control."""

    def __init__(self):
        super().__init__('teleop_converter')
        
        # =================================================================
        # Parameters - Scaling
        # =================================================================
        self.declare_parameter('surge_scale', 20.0)   # Newtons
        self.declare_parameter('sway_scale', 20.0)    # Newtons
        self.declare_parameter('heave_scale', 40.0)   # Newtons
        self.declare_parameter('yaw_scale', 5.0)      # Nm
        self.declare_parameter('roll_scale', 2.0)     # Nm
        self.declare_parameter('pitch_scale', 2.0)    # Nm
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('require_deadman', True)
        
        # =================================================================
        # Parameters - Axis/Button Indices
        # Xbox One S Controller (USB mode, Linux):
        #   Axes: 0=LX, 1=LY, 2=LT, 3=RX, 4=RY, 5=RT, 6=DpadX, 7=DpadY
        #   Buttons: 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, ...
        # More info here: https://wiki.ros.org/joy
        # =================================================================
        self.declare_parameter('axis_left_x', 0)    # Left stick horizontal (Sway)
        self.declare_parameter('axis_left_y', 1)    # Left stick vertical (Surge)
        self.declare_parameter('axis_right_x', 3)   # Right stick horizontal (Yaw)
        self.declare_parameter('axis_lt', 2)        # Left trigger (Heave down)
        self.declare_parameter('axis_rt', 5)        # Right trigger (Heave up)
        self.declare_parameter('axis_dpad_x', 6)    # D-pad horizontal (Roll)
        self.declare_parameter('axis_dpad_y', 7)    # D-pad vertical (Pitch)
        self.declare_parameter('btn_deadman', 4)    # LB button (Deadman)
        
        # Load scaling parameters
        self.surge_scale = self.get_parameter('surge_scale').value
        self.sway_scale = self.get_parameter('sway_scale').value
        self.heave_scale = self.get_parameter('heave_scale').value
        self.yaw_scale = self.get_parameter('yaw_scale').value
        self.roll_scale = self.get_parameter('roll_scale').value
        self.pitch_scale = self.get_parameter('pitch_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.require_deadman = self.get_parameter('require_deadman').value
        
        # Load axis/button indices
        self.axis_left_x = self.get_parameter('axis_left_x').value
        self.axis_left_y = self.get_parameter('axis_left_y').value
        self.axis_right_x = self.get_parameter('axis_right_x').value
        self.axis_lt = self.get_parameter('axis_lt').value
        self.axis_rt = self.get_parameter('axis_rt').value
        self.axis_dpad_x = self.get_parameter('axis_dpad_x').value
        self.axis_dpad_y = self.get_parameter('axis_dpad_y').value
        self.btn_deadman = self.get_parameter('btn_deadman').value
        
        # =================================================================
        # State
        # =================================================================
        self.last_joy_time = None
        self.last_twist_time = None
        self.current_wrench = Wrench()
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        
        # Publishers
        self.wrench_pub = self.create_publisher(
            Wrench, '/controls/effort', 10)
        
        # Timer for publishing at fixed rate and checking timeout
        self.timer = self.create_timer(0.05, self.publish_loop)  # 20 Hz
        
        self.get_logger().info('Teleop Converter initialized')
        self.get_logger().info(f'Scales: surge={self.surge_scale}, heave={self.heave_scale}, yaw={self.yaw_scale}')
        self.get_logger().info(f'Deadman button: {self.btn_deadman}, required: {self.require_deadman}')

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def joy_callback(self, msg: Joy):
        """Process gamepad input."""
        # =================================================================
        # Deadman switch check
        # =================================================================
        deadman_held = True
        if self.require_deadman:
            if len(msg.buttons) > self.btn_deadman:
                deadman_held = bool(msg.buttons[self.btn_deadman])
            else:
                deadman_held = False
        
        if not deadman_held:
            self.current_wrench = Wrench()
            return
        
        # =================================================================
        # Extract axis values
        # =================================================================
        # Left stick: X/Y plane movement (surge/sway)
        surge = 0.0
        sway = 0.0
        if len(msg.axes) > self.axis_left_y:
            surge = self.apply_deadzone(msg.axes[self.axis_left_y])
        if len(msg.axes) > self.axis_left_x:
            sway = self.apply_deadzone(msg.axes[self.axis_left_x])
        
        # Right stick: Yaw only (horizontal axis)
        yaw = 0.0
        if len(msg.axes) > self.axis_right_x:
            yaw = self.apply_deadzone(msg.axes[self.axis_right_x])
        
        # Triggers: Heave (LT = down, RT = up)
        # Triggers typically range from 1.0 (released) to -1.0 (pressed)
        heave = 0.0
        if len(msg.axes) > max(self.axis_lt, self.axis_rt):
            lt = (1.0 - msg.axes[self.axis_lt]) / 2.0  # 0 (released) to 1 (pressed)
            rt = (1.0 - msg.axes[self.axis_rt]) / 2.0  # 0 (released) to 1 (pressed)
            heave = rt - lt  # Positive = up, Negative = down
        
        # D-pad: Roll (left/right) and Pitch (up/down)
        roll = 0.0
        pitch = 0.0
        if len(msg.axes) > self.axis_dpad_x:
            roll = -msg.axes[self.axis_dpad_x]  # +1 = right, -1 = left
        if len(msg.axes) > self.axis_dpad_y:
            pitch = msg.axes[self.axis_dpad_y]  # +1 = up, -1 = down
        
        # =================================================================
        # Only update last_joy_time if there's actual non-zero input
        # This allows keyboard to work when joystick is idle
        # =================================================================
        has_input = (abs(surge) > 0.01 or abs(sway) > 0.01 or abs(heave) > 0.01 or
                     abs(yaw) > 0.01 or abs(roll) > 0.01 or abs(pitch) > 0.01)
        
        if has_input:
            self.last_joy_time = self.get_clock().now()
        
        # =================================================================
        # Build wrench
        # =================================================================
        wrench = Wrench()
        wrench.force.x = surge * self.surge_scale
        wrench.force.y = sway * self.sway_scale
        wrench.force.z = heave * self.heave_scale
        wrench.torque.x = roll * self.roll_scale
        wrench.torque.y = pitch * self.pitch_scale
        wrench.torque.z = yaw * self.yaw_scale
        
        self.current_wrench = wrench

    def twist_callback(self, msg: Twist):
        """Process Twist input from keyboard/Foxglove (fallback if no Joy). (NOT YET IMPLEMENTED)"""
        self.last_twist_time = self.get_clock().now()
        
        # Only use Twist if no recent Joy input
        if self.last_joy_time is not None:
            elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
            if elapsed < self.timeout_sec:
                return  # Joy has priority
        
        # Convert Twist to Wrench (linear mapping)
        wrench = Wrench()
        wrench.force.x = msg.linear.x * self.surge_scale
        wrench.force.y = msg.linear.y * self.sway_scale
        wrench.force.z = msg.linear.z * self.heave_scale
        wrench.torque.x = msg.angular.x * self.roll_scale
        wrench.torque.y = msg.angular.y * self.pitch_scale
        wrench.torque.z = msg.angular.z * self.yaw_scale
        
        self.current_wrench = wrench

    def publish_loop(self):
        """Publish wrench at fixed rate, with timeout check."""
        now = self.get_clock().now()
        
        # Check for input timeout
        joy_stale = True
        twist_stale = True
        
        if self.last_joy_time is not None:
            elapsed = (now - self.last_joy_time).nanoseconds / 1e9
            joy_stale = elapsed > self.timeout_sec
        
        if self.last_twist_time is not None:
            elapsed = (now - self.last_twist_time).nanoseconds / 1e9
            twist_stale = elapsed > self.timeout_sec
        
        # If all inputs are stale, zero output (safety)
        if joy_stale and twist_stale:
            self.current_wrench = Wrench()
        
        # Publish wrench (always publish, even zeros - keeps stream alive)
        self.wrench_pub.publish(self.current_wrench)


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
