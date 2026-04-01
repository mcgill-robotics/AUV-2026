#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Wrench
from propulsion.thrust_mapper_utils import force_to_pwm_thruster
import rclpy.duration
from std_srvs.srv import Trigger
from auv_msgs.srv import SetInt32
from ament_index_python.packages import get_package_share_directory
import yaml
import os

SWEEP_DURATION = 10.0   # seconds for one full sine cycle
SWEEP_RATE_HZ  = 40    # PWM update rate during sweep
PEAK_FORCE_N   = 3.0   # peak force in Newtons for all dry tests

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
        """
        Sweep all thrusters simultaneously through a full sine cycle:
          0 -> +peak -> 0 -> -peak -> 0
        over SWEEP_DURATION seconds at SWEEP_RATE_HZ update rate.
        """
        peak_force = PEAK_FORCE_N
        dt = 1.0 / SWEEP_RATE_HZ
        steps = int(SWEEP_DURATION * SWEEP_RATE_HZ)
        t0 = time.monotonic()

        for i in range(steps):
            t = (i + 1) * dt
            force = peak_force * math.sin(2.0 * math.pi * t / SWEEP_DURATION)

            test_msg = [force_to_pwm_thruster(j + 1, force) for j in range(8)]
            self.publish_thruster(test_msg)

            # Sleep until next step (wall-clock aligned)
            target = t0 + (i + 1) * dt
            sleep_remaining = target - time.monotonic()
            if sleep_remaining > 0:
                self.get_clock().sleep_for(
                    rclpy.duration.Duration(nanoseconds=int(sleep_remaining * 1e9))
                )

        # Ensure clean stop
        self.publish_thruster(reset_msg)

        response.success = True
        response.message = f"All thrusters sine sweep ±{peak_force:.2f} N over {SWEEP_DURATION}s"

        return response

    def optimized_dry_test(self, request, response):
        thruster: int = request.data
        if not 1 <= thruster <= 8:
            response.success = False
            response.message = "Thruster index must be 1–8"
            return response

        optimized_dry_test_msg = reset_msg.copy()
        optimized_dry_test_msg[thruster - 1] = force_to_pwm_thruster(thruster, PEAK_FORCE_N)
        self.publish_thruster(optimized_dry_test_msg)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.publish_thruster(reset_msg)

        response.success = True
        response.message = f"Thruster {request.data} spinning at {PEAK_FORCE_N} N for 1s"

        return response

    def test_thruster(self, request, response):
        """
        Sweep a single thruster through a full sine cycle:
          0 -> +peak -> 0 -> -peak -> 0
        over SWEEP_DURATION seconds at SWEEP_RATE_HZ update rate.
        """
        thruster: int = request.data
        if not 1 <= thruster <= 8:
            response.success = False
            response.message = "Thruster index must be 1–8"
            return response

        peak_force = PEAK_FORCE_N
        dt = 1.0 / SWEEP_RATE_HZ
        steps = int(SWEEP_DURATION * SWEEP_RATE_HZ)
        t0 = time.monotonic()

        for i in range(steps):
            t = (i + 1) * dt
            # sin(2*pi*t/T) gives exactly one full cycle over SWEEP_DURATION
            force = peak_force * math.sin(2.0 * math.pi * t / SWEEP_DURATION)

            test_msg = reset_msg.copy()
            test_msg[thruster - 1] = force_to_pwm_thruster(thruster, force)
            self.publish_thruster(test_msg)

            # Sleep until next step (wall-clock aligned)
            target = t0 + (i + 1) * dt
            sleep_remaining = target - time.monotonic()
            if sleep_remaining > 0:
                self.get_clock().sleep_for(
                    rclpy.duration.Duration(nanoseconds=int(sleep_remaining * 1e9))
                )

        # Ensure clean stop
        self.publish_thruster(reset_msg)

        response.success = True
        response.message = (
            f"Thruster {thruster} sine sweep ±{peak_force:.2f} N "
            f"over {SWEEP_DURATION}s"
        )
        return response

    def re_arm(self, request, response):
        self.get_logger().info("Re-arming thrusters...")
        response.success = True
        response.message = "Re-arming thrusters..."

        arming_msg = ([1540] * 8)

        self.publish_thruster(reset_msg)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.publish_thruster(arming_msg)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
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