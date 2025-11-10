from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from ros2_ws.src.vision.vision.image_enhancement import enhancement_algorithms as enhance

class EnhanceNode(Node):
    def __init__(self,node_name:str,input_topic:str,output_topic:str,enhancer:enhance.ImageEnhancer):
        super().__init__(node_name)
        self.subscription = self.create_subscription(
			Image, # Image message type
			input_topic, # Topic name
			self.enhancement_callback, # Callback, called on message received
			10 # QoS: if received message > this #, start dropping oldest received ones
		)
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.get_logger().info(f"EnhanceNode initialized with input topic: {input_topic} and output topic: {output_topic}")
        self.enhancer = enhancer
        self.get_logger().info(f"Using Enhancer: {self.enhancer}")
        self.br = CvBridge()
        
    def enhancement_callback(self, msg):
		# ROS2 -> OpenCV
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        enhanced_image = self.enhancer.enhance(cv_image)
		# OpenCV -> ROS2
        enhanced_msg = self.br.cv2_to_imgmsg(enhanced_image, encoding='bgr8')
		# Publish enhanced image
        self.publisher.publish(enhanced_msg)