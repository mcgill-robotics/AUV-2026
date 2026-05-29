import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import select
import termios
import tty

msg = """
Reading from the keyboard and Publishing to Joy
---------------------------
Toggles:
  1 : Toggle Controls Mode (LB)
  2 : Toggle Propulsion Mode (RB)

Movement:
  W/S : Surge Forward/Backward
  A/D : Sway Left/Right
  E/Q : Heave Up/Down

  J/L : Yaw Left/Right
  I/K : Pitch Forward/Backward
  U/O : Roll Left/Right

  Space : Stop all movement

CTRL-C to quit
"""

moveBindings = {
    'w': (1.0, 'surge'),
    's': (-1.0, 'surge'),
    'a': (1.0, 'sway'),
    'd': (-1.0, 'sway'),
    'e': (1.0, 'heave'),
    'q': (-1.0, 'heave'),
    'j': (1.0, 'yaw'),
    'l': (-1.0, 'yaw'),
    'i': (1.0, 'pitch'), 
    'k': (-1.0, 'pitch'),
    'u': (1.0, 'roll'),  
    'o': (-1.0, 'roll'), 
}

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.05, self.publish_joy) # 20 Hz
        
        # Axis mappings based on teleop.yaml
        self.axis_map = {
            'sway': 0,
            'surge': 1,
            'yaw': 2,
            'pitch': 3,
            'heave_down': 4,
            'heave_up': 5
        }
        
        # Button mappings
        self.btn_lb = 9
        self.btn_rb = 10
        self.btn_dpad_up = 11
        self.btn_dpad_down = 12
        self.btn_dpad_left = 13
        self.btn_dpad_right = 14
        
        self.axes = [0.0] * 8
        self.buttons = [0] * 15
        
        # Release trigger axes to 1.0 (default for joystick)
        self.axes[self.axis_map['heave_down']] = 1.0
        self.axes[self.axis_map['heave_up']] = 1.0
        
        self.last_key_time = self.get_clock().now()
        
        if not sys.stdin.isatty():
            self.get_logger().fatal("This node requires a TTY to read keyboard input")
            self.get_logger().fatal("You cannot launch it via ros2 launch without a terminal prefix")
            self.get_logger().fatal("Please run it directly in a dedicated terminal:")
            self.get_logger().fatal("    ros2 run teleop keyboard_to_joy")
            sys.exit(1)
            
        self.settings = termios.tcgetattr(sys.stdin)
        print(msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                # Read arrow key escape sequences
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_joy(self):
        key = self.getKey()
        
        if key in moveBindings.keys():
            self.last_key_time = self.get_clock().now()
            val, cmd = moveBindings[key]
            if cmd == 'surge':
                self.axes[self.axis_map['surge']] = val
            elif cmd == 'sway':
                self.axes[self.axis_map['sway']] = val
            elif cmd == 'yaw':
                self.axes[self.axis_map['yaw']] = val
            elif cmd == 'heave':
                # Map to triggers: 1.0 is released, -1.0 is pressed fully
                if val > 0:
                    self.axes[self.axis_map['heave_up']] = -1.0
                    self.axes[self.axis_map['heave_down']] = 1.0
                else:
                    self.axes[self.axis_map['heave_up']] = 1.0
                    self.axes[self.axis_map['heave_down']] = -1.0
            elif cmd == 'pitch':
                # D-pad simulation
                if val > 0:
                    self.buttons[self.btn_dpad_up] = 1
                    self.buttons[self.btn_dpad_down] = 0
                else:
                    self.buttons[self.btn_dpad_up] = 0
                    self.buttons[self.btn_dpad_down] = 1
            elif cmd == 'roll':
                # D-pad simulation
                if val > 0:
                    self.buttons[self.btn_dpad_left] = 1
                    self.buttons[self.btn_dpad_right] = 0
                else:
                    self.buttons[self.btn_dpad_left] = 0
                    self.buttons[self.btn_dpad_right] = 1
        elif key == ' ':
            self.last_key_time = self.get_clock().now()
            # Stop all
            self.axes[self.axis_map['surge']] = 0.0
            self.axes[self.axis_map['sway']] = 0.0
            self.axes[self.axis_map['yaw']] = 0.0
            self.axes[self.axis_map['heave_up']] = 1.0
            self.axes[self.axis_map['heave_down']] = 1.0
            self.buttons[self.btn_dpad_up] = 0
            self.buttons[self.btn_dpad_down] = 0
            self.buttons[self.btn_dpad_left] = 0
            self.buttons[self.btn_dpad_right] = 0
        elif key == '1':
            # Toggle LB (Controls Mode)
            self.buttons[self.btn_lb] = 1 - self.buttons[self.btn_lb]
            if self.buttons[self.btn_lb]:
                self.buttons[self.btn_rb] = 0  # Enforce mutual exclusivity
                print("Controls Mode ENABLED")
            else:
                print("Controls Mode DISABLED")
        elif key == '2':
            # Toggle RB (Propulsion Mode)
            self.buttons[self.btn_rb] = 1 - self.buttons[self.btn_rb]
            if self.buttons[self.btn_rb]:
                self.buttons[self.btn_lb] = 0  # Enforce mutual exclusivity
                print("Propulsion Mode ENABLED")
            else:
                print("Propulsion Mode DISABLED")
        elif key == '\x03':
            # CTRL-C
            sys.exit(0)
            
        # Spring back to zero (simulate joystick release) if no key pressed recently.
        # Timeout increased to 0.5s to completely override any OS-level auto-repeat delays or slow tapping.
        if (self.get_clock().now() - self.last_key_time).nanoseconds / 1e9 > 0.5:
            if self.axes[self.axis_map['surge']] != 0.0 or self.axes[self.axis_map['sway']] != 0.0 or self.axes[self.axis_map['yaw']] != 0.0:
                print("Keyboard idle timeout (0.5s). Springing axes back to 0.0.")
            
            self.axes[self.axis_map['surge']] = 0.0
            self.axes[self.axis_map['sway']] = 0.0
            self.axes[self.axis_map['yaw']] = 0.0
            self.axes[self.axis_map['heave_up']] = 1.0
            self.axes[self.axis_map['heave_down']] = 1.0
            self.buttons[self.btn_dpad_up] = 0
            self.buttons[self.btn_dpad_down] = 0
            self.buttons[self.btn_dpad_left] = 0
            self.buttons[self.btn_dpad_right] = 0
            
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = 'keyboard'
        joy_msg.axes = self.axes
        joy_msg.buttons = self.buttons
        self.pub.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
