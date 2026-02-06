#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Wrench
from propulsion.thrust_mapper_utils import force_to_pwm_thruster
from time import sleep
from std_srvs.srv import Trigger, SetInt32

MAX_FWD_FORCE = 4 * 9.81 # Numbers from drytest in AUV-2025
MAX_BWD_FORCE = -2 * 9.81 # Numbers from drytest in AUV-2025


force_amt = 0.1

reset_msg = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

class DryTestNode(Node):
    def __init__(self):
        super().__init__('dry_test_node')
        self.thruster_microseconds_pub = self.create_publisher(Int16MultiArray, '/propulsion/microseconds', 1)
        self.forces_pub = self.create_publisher(Wrench, '/controls/effort', 1)
        self.get_logger().info("Dry Test Node Initialized")

        self.create_service(Trigger, "~/re_arm", self.re_arm)
        self.create_service(SetInt32, "~/optimized_dry_test", self.optimized_dry_test)
        self.create_service(Trigger, "~/simultaneous_test", self.simultaneous_forwards_test)

    def publish_thruster(self, msg):
        #self.get_logger().info(f"Publishing thruster command: {msg}")
        ros_msg = Int16MultiArray()
        ros_msg.data = msg
        self.thruster_microseconds_pub.publish(ros_msg)

    def publish_force(self, msg):
        #self.get_logger().info(f"Publishing force command: {msg}")
        ros_msg = Wrench()
        ros_msg.force = msg.force
        ros_msg.torque = msg.torque
        self.forces_pub.publish(ros_msg)    

    def reset_thrusters(self):
        self.publish_thruster(reset_msg)
        self.get_logger().info("Safely shutting down thrusters")

    def simultaneous_forwards_test(self, request, response):
        forward_test_msg = [force_to_pwm_thruster(i + 1, force_amt * MAX_FWD_FORCE) for i in range(8)]
        self.publish_thruster(forward_test_msg)
        sleep(1)
        self.publish_thruster(reset_msg)

        response.success = True
        response.message = "all thrusters spinning at " + str(100 * force_amt) + "% max forwards force for 1s"

        return response

    def optimized_dry_test(self, request, response):
        thruster: int = request.data
        if not 1 <= thruster <= 8:
            response.success = False
            response.message = "Thruster index must be 1–8"
            return response

        optimized_dry_test_msg = reset_msg.copy()
        optimized_dry_test_msg[thruster - 1] = force_to_pwm_thruster(thruster, force_amt * MAX_FWD_FORCE)
        self.publish_thruster(optimized_dry_test_msg)
        sleep(1)
        self.publish_thruster(reset_msg)

        response.success = True
        response.message = "Thruster " + str(request.data) + " spinning at " + str(100 * force_amt) + "% max forwards force for 1s"

        return response

    def re_arm(self, request, response):
        self.get_logger().info("Re-arming thrusters...")
        response.success = True
        response.message = "Re-arming thrusters..."

        arming_msg = ([1540] * 8)

        self.publish_thruster(reset_msg)
        sleep(1)
        self.publish_thruster(arming_msg)
        sleep(1)
        self.publish_thruster(reset_msg)

        return response

def dry_test(self):
    while rclpy.ok():
        print("1. select thruster to test")
        print("2. test thrusters simultaneously")
        print("3. re-arm")
        print("4. Keyboard control")
        print("5. exit")
        choice = input()
        print("You selected option " + choice)
        if choice == "1":
            while True:
                choice = input("Select thruster to test (1-8) or any other key to go back:\n")
                try:
                    choice = int(choice)
                except:
                    print("Invalid input, returning to main menu.")
                    break
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
                desired_effort.force.x = 0.0
                desired_effort.force.y = 0.0
                desired_effort.force.z = 0.0
                desired_effort.torque.x = 0.0
                desired_effort.torque.y = 0.0
                desired_effort.torque.z = 0.0
                key = input("Press key to control, then ENTER to send command (b to stop) (y to exit)\n")
                if key == "y":
                    print("Exiting keyboard control mode...")
                    self.publish_thruster(reset_msg)
                    break
                if key == "b" or key == "":
                    print("Stopping all thrusters")
                    self.publish_thruster(reset_msg)
                    continue
                if key == "w":
                    desired_effort.force.x = (
                        desired_effort.force.x + force_amt * MAX_FWD_FORCE
                    )
                if key == "s":
                    desired_effort.force.x = (
                        desired_effort.force.x + force_amt * MAX_BWD_FORCE
                    )
                if key == "a":
                    desired_effort.force.y = (
                        desired_effort.force.y + force_amt * MAX_FWD_FORCE
                    )
                if key == "d":
                    desired_effort.force.y = (
                        desired_effort.force.y + force_amt * MAX_BWD_FORCE
                    )
                if key == "q":
                    desired_effort.force.z = (
                        desired_effort.force.z + force_amt * MAX_FWD_FORCE
                    )
                if key == "e":
                    desired_effort.force.z = (
                        desired_effort.force.z + force_amt * MAX_BWD_FORCE
                    )
                if key == "o":
                    desired_effort.torque.x = (
                        desired_effort.torque.x + force_amt * MAX_FWD_FORCE
                    )
                if key == "u":
                    desired_effort.torque.x = (
                        desired_effort.torque.x + force_amt * MAX_BWD_FORCE
                    )
                if key == "i":
                    desired_effort.torque.y = (
                        desired_effort.torque.y + force_amt * MAX_FWD_FORCE
                    )
                if key == "k":
                    desired_effort.torque.y = (
                        desired_effort.torque.y + force_amt * MAX_BWD_FORCE
                    )
                if key == "j":
                    desired_effort.torque.z = (
                        desired_effort.torque.z + force_amt * MAX_FWD_FORCE
                    )
                if key == "l":
                    desired_effort.torque.z = (
                        desired_effort.torque.z + force_amt * MAX_BWD_FORCE
                    )
                self.publish_force(desired_effort)
                print("Force (x=" + str(desired_effort.force.x) + ", y=" + str(desired_effort.force.y) + ", z=" + str(desired_effort.force.z) + \
                      "), Torque (x=" + str(desired_effort.torque.x) + ", y=" + str(desired_effort.torque.y) + ", z=" + str(desired_effort.torque.z) + ")")

        if choice == "5":
            while True:
                thruster_num = input("Select thruster to test (1-8) or any other key to go back:\n")
                try:
                    thruster_num = int(thruster_num)
                except:
                    print("Invalid input, returning to main menu.")
                    break
                if not (1 <= thruster_num and thruster_num <= 8):
                    print("Invalid input, returning to main menu.")
                    break
                duration = input("Select duration to run test (in seconds):\n")
                try:
                    duration = float(duration)
                except:
                    print("Invalid input")
                    continue
                if duration <= 0:
                    print("Invalid input")
                    continue
                print("Thruster " + str(thruster_num) + " spinning at " + str(100 * force_amt) + "% max forwards force for " + str(duration) + "s")
                custom_test_msg = reset_msg.copy()
                custom_test_msg[thruster_num - 1] = force_to_pwm_thruster(thruster_num, force_amt * MAX_FWD_FORCE * thruster_mount_dirs[thruster_num - 1])
                self.publish_thruster(custom_test_msg)
                sleep(duration)
                self.publish_thruster(reset_msg)
        else:
            self.publish_thruster(reset_msg)
            print("Thank you for using the dry test program. Exiting...")
            break


def main(args=None):
    rclpy.init(args=args)
    dry_test_node = DryTestNode()
    
    try:
        rclpy.spin(dry_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        dry_test_node.reset_thrusters()
        dry_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()