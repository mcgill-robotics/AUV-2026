import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion, PoseStamped


class SetpointToFoxgloveNode(Node):
    def __init__(self):
        super().__init__('setpoint_to_foxglove_node')
        
        self.declare_parameter('frame_id', 'pool')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/foxglove/pose_setpoint', 10)
        
        # Subscribers
        self.create_subscription(Float64, '/controls/x_setpoint', self.x_callback, 10)
        self.create_subscription(Float64, '/controls/y_setpoint', self.y_callback, 10)
        self.create_subscription(Float64, '/controls/depth_setpoint', self.z_callback, 10)
        self.create_subscription(Quaternion, '/controls/quaternion_setpoint', self.q_callback, 10)
        
        self.get_logger().info("Setpoint to Foxglove converter started.")
        self.get_logger().info(f"Publishing unified PoseStamped to: /foxglove/pose_setpoint (frame: {self.frame_id})")

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        
        msg.pose.orientation.x = self.qx
        msg.pose.orientation.y = self.qy
        msg.pose.orientation.z = self.qz
        msg.pose.orientation.w = self.qw
        
        self.pose_pub.publish(msg)

    def x_callback(self, msg):
        self.x = msg.data
        self.publish_pose()
        
    def y_callback(self, msg):
        self.y = msg.data
        self.publish_pose()
        
    def z_callback(self, msg):
        self.z = -msg.data
        self.publish_pose()
        
    def q_callback(self, msg):
        self.qx = msg.x
        self.qy = msg.y
        self.qz = msg.z
        self.qw = msg.w
        self.publish_pose()


def main(args=None):
    rclpy.init(args=args)
    node = SetpointToFoxgloveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
