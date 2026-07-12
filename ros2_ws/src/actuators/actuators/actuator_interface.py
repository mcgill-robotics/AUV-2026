import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class ActuatorInterface(Node):
    def __init__(self):
        super().__init__("actuator_interface")

        #publishers for hardware topics
        self.grabber_pub = self.create_publisher(UInt8, "/actuators/hw/grabber", 10)
        self.torpedo_pub = self.create_publisher(UInt8, "/actuators/hw/torpedo", 10) 

        #subscriptions for incoming commands
        self.create_subscription(UInt8, "/actuators/grabber", self.grabber_callback, 10)
        self.create_subscription(UInt8, "/actuators/torpedo", self.torpedo_callback, 10)

        self.get_logger().info("Actuator interface ready")

    def grabber_callback(self, msg: UInt8):
        if msg.data not in (0, 1):
            self.get_logger().warning(f"Ignoring invalid grabber command: {msg.data}")
            return

        self.grabber_pub.publish(msg)
        self.get_logger().info(f"Forwarded grabber command: {msg.data}")

    def torpedo_callback(self, msg: UInt8):
        if msg.data not in (0, 1, 2):
            self.get_logger().warning(f"Ignoring invalid torpedo command: {msg.data}")
            return

        self.torpedo_pub.publish(msg)
        self.get_logger().info(f"Forwarded torpedo command: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
