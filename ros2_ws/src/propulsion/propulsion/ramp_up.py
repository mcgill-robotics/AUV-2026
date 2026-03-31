#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Wrench
from propulsion.thrust_mapper_utils import force_to_pwm_thruster
from time import sleep

reset_msg = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

class DryTestNode(Node):
    def __init__(self):
        super().__init__('dry_test_node')
        self.thruster_microseconds_pub = self.create_publisher(Int16MultiArray, '/propulsion/microseconds', 1)
        self.get_logger().info("Dry Test Node Initialized")

    def publish_thruster(self, msg):
        #self.get_logger().info(f"Publishing thruster command: {msg}")
        ros_msg = Int16MultiArray()
        ros_msg.data = msg
        self.thruster_microseconds_pub.publish(ros_msg)  

    def reset_thrusters(self):
        self.publish_thruster(reset_msg)
        self.get_logger().info("Safely shutting down thrusters")

def dry_test(self):
    while rclpy.ok():
        print("Select which thruster to test (1-8)")
        current_thruster = int(input())
        print("You selected thruster " + str(current_thruster))
        if not (current_thruster in [1, 2, 3, 4, 5, 6, 7, 8]):
            print("Invalid choice. Please select a thruster between 1 and 8.")
            continue
        
        print("Press \"a\" to choose an amount to increment by \n" \
		"\"b\" to select a specific PWM value \n"
        "or any other key to exit")
        choice = input()
        
        if choice == "a":
            print("Choose a PWM value for thruster " + choice + " between 1100 and 1900")
            thruster_value = int(input())
            original_matrix = reset_msg.copy()
            original_matrix[current_thruster - 1] = thruster_value
            key = input("Press y key to stop\n")
            while True:
                if key == "y":
                    print("Exiting keyboard control mode...")
                    self.publish_thruster(reset_msg)
                    break
                else: 
                    self.publish_thruster(original_matrix)
        elif choice == "b":
            increment_matrix = reset_msg.copy()
            print("Choose an integer to increment thruster PWM by")
            thruster_increment = int(input())
            print("Choose a start value between 1100 and 1900 for thruster " + str(current_thruster))
            increment_matrix[current_thruster - 1] = int(input())
            print("NOTE: Uses thrust mapper\n\
                    > Press i to increase thruster PWM by " + str(thruster_increment) + "\n\
                    > Press j to decrease thruster PWM by " + str(thruster_increment) + "\n\
                    > Press b or ENTER to stop\n\
					> Press y to exit\n")
            while True:
                key = input("Press key to control, then ENTER to send command (b to stop) (y to exit)\n")
                if key == "y":
                    print("Exiting test mode...")
                    self.publish_thruster(reset_msg)
                    break
                if key == "b" or key == "":
                    print("Stopping all thrusters")
                    self.publish_thruster(reset_msg)
                    continue
                if key == "i":
                    increment_matrix[current_thruster - 1] = min(increment_matrix[current_thruster - 1] + thruster_increment, 1900)
                if key == "j":
                    increment_matrix[current_thruster - 1] = max(increment_matrix[current_thruster - 1] - thruster_increment, 1100)
                self.publish_thruster(increment_matrix)
                print("Current PWM for thruster " + str(current_thruster) + ": " + str(increment_matrix[current_thruster - 1]))
        else:
            self.publish_thruster(reset_msg)
            print("Thank you for using the dry test program. Exiting...")
            break


def main(args=None):
    rclpy.init(args=args)
    dry_test_node = DryTestNode()
    try:
        dry_test(dry_test_node)
    except:
        pass
    finally:
        dry_test_node.publish_thruster(reset_msg)
        dry_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()