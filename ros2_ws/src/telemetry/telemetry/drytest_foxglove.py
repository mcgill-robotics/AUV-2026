#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Wrench
from propulsion.thrust_mapper_utils import force_to_pwm_thruster
from time import sleep
from std_srvs.srv import Trigger
from auv_msgs.srv import SetInt32
from ament_index_python.packages import get_package_share_directory
import yaml
import os

MAX_FWD_FORCE = 4 * 9.81 # Numbers from drytest in AUV-2025
MAX_BWD_FORCE = -2 * 9.81 # Numbers from drytest in AUV-2025


force_amt = 0.1

reset_msg = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]


def load_topics_config():
    """Load centralized topic config from telemetry/config/topics.yaml."""
    config_path = os.path.join(
        get_package_share_directory('telemetry'), 'config', 'topics.yaml'
    )
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


class DryTestNode(Node):
    def __init__(self):
        super().__init__('dry_test_node')

        # Load topics from centralized config
        topics = load_topics_config()
        microseconds_topic = topics['propulsion']['microseconds']
        effort_topic = topics['controls']['effort']

        self.thruster_microseconds_pub = self.create_publisher(Int16MultiArray, microseconds_topic, 1)
        self.forces_pub = self.create_publisher(Wrench, effort_topic, 1)
        self.get_logger().info(f"Dry Test Node Initialized (publishing to {microseconds_topic}, {effort_topic})")

        self.create_service(Trigger, "~/re_arm", self.re_arm)
        self.create_service(SetInt32, "~/optimized_dry_test", self.optimized_dry_test)
        self.create_service(SetInt32, "~/test_thruster", self.test_thruster)
        self.create_service(Trigger, "~/simultaneous_test", self.simultaneous_forwards_test)

    def publish_thruster(self, msg):
        ros_msg = Int16MultiArray()
        ros_msg.data = msg
        self.thruster_microseconds_pub.publish(ros_msg)

    def publish_force(self, msg):
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

    def test_thruster(self, request, response):
        """
        Rotates a single thruster forward for 1s, then backward for 1s, then stops.
        """
        thruster: int = request.data
        if not 1 <= thruster <= 8:
            response.success = False
            response.message = "Thruster index must be 1–8"
            return response
            
        # Forward
        test_msg = reset_msg.copy()
        test_msg[thruster - 1] = force_to_pwm_thruster(thruster, force_amt * MAX_FWD_FORCE)
        self.publish_thruster(test_msg)
        sleep(1)
        
        # Backward
        test_msg[thruster - 1] = force_to_pwm_thruster(thruster, force_amt * MAX_BWD_FORCE)
        self.publish_thruster(test_msg)
        sleep(1)
        
        # Stop
        self.publish_thruster(reset_msg)
        
        response.success = True
        response.message = f"Thruster {thruster} forward 1s, backward 1s, then stopped"
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