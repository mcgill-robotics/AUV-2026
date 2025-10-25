#!/usr/bin/env python3
'''
Initial Commit by Stuart, needs to be reviewed and tested, still learning how ros2 works
ROS 2 Node for interactively testing and adjusting the thrust allocation matrix.
Goals:
1. Minimize pitch occurring from surge command by adjusting 'e' parameter.
2. Update the procedure to also measure Center of Mass (CoM) offset in the XY plane (gx, gy).

Converted from ROS 1 (mechtest2.py) to ROS 2 (class-based node) for AUV-2026.
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import time

try:
    from auv_msgs.msg import ThrusterMicroseconds
#verify the package name 'auv_msgs' is correct 
except ImportError:
    print("WARNING: Cannot import ThrusterMicroseconds. Check package name")

# --- CONSTANTS ---
# Standard neutral PWM value for the thrusters.
NEUTRAL_PWM = 1500
# Duration to command the force (in seconds)
TEST_DURATION = 20.0
# Frequency (Hz) at which to publish the wrench command
PUBLISH_RATE_HZ = 10

class MechTestNode(Node):
    """
    A ROS 2 Node to command a test wrench and allow interactive CoM adjustment.
    """
    def __init__(self):
        # 1. Initialize the Node with its name
        # The node name is registered in the ROS 2 network.
        super().__init__('allocation_test_node')
        self.get_logger().info("MechTestNode started. Ready for interactive testing.")

        # 2. Declare the Publishers
        # ROS 2 publishers are created with create_publisher(MsgType, topic_name, QoS_profile)
        # For control commands, a small queue_size (like 10) is often used for the default profile.

        self.pwm_pub = self.create_publisher(
            ThrusterMicroseconds, 
#verify this topic name 
            '/propulsion/microseconds', 
            10
        )

        # Publisher for the test command (sends Wrench to the Thrust Mapper node)
        self.effort_pub = self.create_publisher(
            Wrench, 
#verify this topic name is correct for Wrench input to the mapper
            '/controls/effort', 
            10
        ) 
        
        # 3. Define Constants/Commands
        self.reset_cmd = ThrusterMicroseconds(microseconds=[NEUTRAL_PWM] * 8) 
        
        # 4. Declare the CoM offset parameters (gx, gy)
#These are the new parameters we want to tune
        self.declare_parameter('gx_offset', 0.0)
        self.declare_parameter('gy_offset', 0.0)
        self.get_logger().info("CoM parameters 'gx_offset' and 'gy_offset' declared (default 0.0).")

        # Give time for the connection to fully establish
        time.sleep(1.0) 
        
        # 5. Start the main interactive loop immediately
        self.run_allocation_test()


    def run_allocation_test(self):
        """Main loop to interactively test the thrust allocation matrix."""

        # Outer loop allows repeating the test
        while rclpy.ok():
            
            # Safety reset before asking for input
            self.pwm_pub.publish(self.reset_cmd) 
            
            user_input = input("\n[INPUT] Press ENTER to start new allocation test, or type 'q' to quit: ")
            if user_input.lower() == 'q':
                print("[INFO] Exiting test loop. Sending final neutral command.")
                self.pwm_pub.publish(self.reset_cmd)
                break

            # Get the desired force from the user
            try:
                # We must command a forward force (Surge) to test the Pitch coupling
                surge_force = float(input("\n[INPUT] Specify a FORWARD (Surge) force in Newtons (e.g., 5.0): "))
            except ValueError:
                self.get_logger().error("Invalid input. Using default force of 5.0 N.")
                surge_force = 5.0
            
            # --- PHASE 1: COMMAND PURE SURGE ---
            self.get_logger().info(f"Commanding pure Surge force: {surge_force:.2f} N for {TEST_DURATION} seconds...")

            # 1. Create the Wrench message
            test_wrench = Wrench()
            
            # 2. Set the Surge (X) Force component
            test_wrench.force.x = surge_force 
            
            # All other 5 components are implicitly zero (0.0)

            # 3. Publish the Wrench command using a simple loop and standard sleep
            start_time_sec = self.get_clock().now().nanoseconds / 1e9 # Get time in seconds
            
            # Calculate the sleep time needed to maintain the desired rate
            sleep_duration = 1.0 / PUBLISH_RATE_HZ
            
            while ((self.get_clock().now().nanoseconds / 1e9 - start_time_sec) < TEST_DURATION 
                   and rclpy.ok()):
                
                self.effort_pub.publish(test_wrench)
                time.sleep(sleep_duration) # Use standard blocking sleep for simple scripts

            # --- PHASE 2: RESET AND OBSERVE ---
            self.pwm_pub.publish(self.reset_cmd)
            self.get_logger().info("Test complete. Thrusters reset to neutral.")
            self.get_logger().info(
                "[OBSERVE] Check the AUV's pitch tendency. If it pitches, the 'e' parameter needs adjustment."
                "\n[NEW FOCUS] Use this time to observe any side-to-side (Sway/Yaw) movement to tune the new 'gx_offset' and 'gy_offset' parameters in the launch file/config."
            )
            
            # Optional: Display current CoM offsets from ROS parameters for context
            gx = self.get_parameter('gx_offset').value
            gy = self.get_parameter('gy_offset').value
            print(f"Current CoM Offsets being used: gx={gx:.4f} m, gy={gy:.4f} m")


def main(args=None):
    # Standard boilerplate for ROS 2 Python nodes
    rclpy.init(args=args)
    
    # Instantiate your node class
    node = MechTestNode()
    
    # Run the main node loop (in this case, the interactive prompt)
    # Since run_allocation_test() is blocking, rclpy.spin_once() is used just to process
    # any last ROS events if needed before shutdown, but the main logic is in the loop.
    try:
        # The MechTestNode's __init__ calls run_allocation_test() which blocks until 'q'
        pass 
    except KeyboardInterrupt:
        pass
    
    # Ensure all resources are properly released
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()