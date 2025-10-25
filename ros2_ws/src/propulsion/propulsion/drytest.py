#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from auv_msgs.msg import ThrusterMicroseconds
from geometry_msgs.msg import Wrench
import keyboard
from propulsion.thrust_mapper_utils import force_to_pwm_thruster

MAX_FWD_FORCE = 4 * 9.81 # Numbers from drytest in AUV-2025
MAX_BWD_FORCE = -2 * 9.81 # Numbers from drytest in AUV-2025

thruster_mount_dirs = [1, 1, 1, 1, 1, 1, 1, 1] # THIS NEEDS TO BE SET BASED ON MOUNTING ORIENTATION

force_amt = 0.1

reset_msg = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

class DryTestNode(Node):
    def __init__(self):
        super().__init__('dry_test_node')
        self.thruster_microseconds_pub = self.create_publisher(ThrusterMicroseconds, '/propulsion/microseconds', 1)
        self.forces_pub = self.create_publisher(Wrench, '/controls/effort', 1)
        self.rate = self.create_rate(1.0)
        rclpy.on_shutdown(self.reset_thrusters)
        self.get_logger().info("Dry Test Node Initialized")

    def publish_thruster(self, msg):
        self.get_logger().info(f"Publishing thruster command: {msg}")
        self.thruster_microseconds_pub.publish(ThrusterMicroseconds(msg))
    
    def publish_force(self, msg):
        self.get_logger().info(f"Publishing force command: {msg}")
        self.forces_pub.publish(Wrench(msg))

    def reset_thrusters(self):
        self.publish_thruster(reset_msg)
        self.get_logger().info("Safely shutting down thrusters")

    def simultaneous_forwards_test(self):
        while rclpy.ok():
            self.get_logger().info("all thrusters spinning at " + str(100 * force_amt) + "% max forwards force for 1s")
            forward_test_msg = [force_to_pwm_thruster(i + 1, force_amt * MAX_FWD_FORCE * thruster_mount_dirs[i]) for i in range(8)]
            self.publish_thruster(forward_test_msg)
            self.rate.sleep()
            self.publish_thruster(reset_msg)

            choice = input("Press 1 to repeat test\nPress 2 to proceed\n")
            if choice != "1":
                break

    def optimized_dry_test(self, t):
        print("Thruster " + str(t) + " spinning at " + str(100 * force_amt) + "% max forwards force for 1s")
        optimized_dry_test_msg = reset_msg.copy()
        optimized_dry_test_msg[t - 1] = force_to_pwm_thruster(t, force_amt * MAX_FWD_FORCE * thruster_mount_dirs[t - 1])
        self.publish_thruster(optimized_dry_test_msg)
        self.rate.sleep()
        self.publish_thruster(reset_msg)

    def re_arm(self):
        self.get_logger().info("Re-arming thrusters...")
        self.rate.sleep()

        arming_msg = ThrusterMicroseconds([1540] * 8)

        self.publish_thruster(reset_msg)
        self.rate.sleep()
        self.publish_thruster(arming_msg)
        self.rate.sleep()
        self.publish_thruster(reset_msg)

    def dry_test(self):
        while rclpy.ok():
            choice = input("========== Thrusters Test ========== \
                           (refer to images in dry-test/images for labelled diagrams of thrusters on the AUV)\n\
                           Please select a testing option by pressing the corresponding key:\n\
                           1. select thruster to test\n\
                           2. test thrusters simultaneously\n\
                           3. re-arm\n\
                           4. Keyboard control\n\
                           5. exit test \n")

            if choice == "1":
                while True:
                    choice = int(input("Select thruster to test (1-8) or any other key to go back:\n"))
                    if 1 <= choice and choice <= 8:
                        self.optimized_dry_test(choice)
                    else:
                        break
            elif choice == "2":
                self.simultaneous_forwards_test()
            elif choice == "3":
                self.re_arm()
            elif choice == "4":
                print("NOTE: Uses thrust mapper\n\
                       > WASD for SURGE/SWAY (forward/backward, left/right)\n\
                       > Q/E for UP/DOWN\n\
                       > IJKL for PITCH/YAW\n\
                       > U/O for ROLL\n\
                       > ESC to exit\n")
                while True:
                    desired_effort = Wrench()
                    desired_effort.force.x = 0
                    desired_effort.force.y = 0
                    desired_effort.force.z = 0
                    desired_effort.torque.x = 0
                    desired_effort.torque.y = 0
                    desired_effort.torque.z = 0

                    if keyboard.is_pressed("esc"):
                        break
                    if keyboard.is_pressed("w"):
                        desired_effort.force.x = (
                            desired_effort.force.x + force_amt * MAX_FWD_FORCE
                        )
                    if keyboard.is_pressed("s"):
                        desired_effort.force.x = (
                            desired_effort.force.x + force_amt * MAX_BWD_FORCE
                        )
                    if keyboard.is_pressed("a"):
                        desired_effort.force.y = (
                            desired_effort.force.y + force_amt * MAX_FWD_FORCE
                        )
                    if keyboard.is_pressed("d"):
                        desired_effort.force.y = (
                            desired_effort.force.y + force_amt * MAX_BWD_FORCE
                        )
                    if keyboard.is_pressed("q"):
                        desired_effort.force.z = (
                            desired_effort.force.z + force_amt * MAX_FWD_FORCE
                        )
                    if keyboard.is_pressed("e"):
                        desired_effort.force.z = (
                            desired_effort.force.z + force_amt * MAX_BWD_FORCE
                        )
                    if keyboard.is_pressed("o"):
                        desired_effort.torque.y = (
                            desired_effort.torque.x + force_amt * MAX_FWD_FORCE
                        )
                    if keyboard.is_pressed("u"):
                        desired_effort.torque.y = (
                            desired_effort.torque.x + force_amt * MAX_BWD_FORCE
                        )
                    if keyboard.is_pressed("i"):
                        desired_effort.torque.y = (
                            desired_effort.torque.y + force_amt * MAX_FWD_FORCE
                        )
                    if keyboard.is_pressed("k"):
                        desired_effort.torque.y = (
                            desired_effort.torque.y + force_amt * MAX_BWD_FORCE
                        )
                    if keyboard.is_pressed("j"):
                        desired_effort.torque.z = (
                            desired_effort.torque.z + force_amt * MAX_FWD_FORCE
                        )
                    if keyboard.is_pressed("l"):
                        desired_effort.torque.z = (
                            desired_effort.torque.z + force_amt * MAX_BWD_FORCE
                        )
                    self.publish_force(desired_effort)
                    self.rate.sleep()
            else:
                break

def main(args=None):
    rclpy.init(args=args)
    dry_test_node = DryTestNode()
    rclpy.spin(dry_test_node)
    dry_test_node.dry_test()
    dry_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()