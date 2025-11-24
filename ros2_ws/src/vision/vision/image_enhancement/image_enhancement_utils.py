from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from cv2 import error as cv2error

from vision.image_enhancement import enhancement_algorithms as enhance

class EnhanceNode(Node):
    def __init__(self,node_name:str,input_topic:str,output_topic:str,enhancer:enhance.ImageEnhancer):
        super().__init__(node_name)
        
        self.declare_parameter("input_topic", input_topic)
        self.declare_parameter("output_topic", output_topic)

        # Get parameters, include fallback to provided arguments
        self.input_topic = self.get_parameter("input_topic").value or input_topic
        self.output_topic = self.get_parameter("output_topic").value or output_topic

        self.subscription = self.create_subscription(
			Image, # Image message type
			self.input_topic, # Topic name
			self.enhancement_callback, # Callback, called on message received
			10 # QoS: if received message > this #, start dropping oldest received ones
		)
        self.publisher = self.create_publisher(Image, self.output_topic, 10)
        self.get_logger().info(
            f"EnhanceNode initialized with \
            input topic: {self.input_topic} and \
            output topic: {self.output_topic}"
        )
        self.enhancer = enhancer
        self.get_logger().info(f"Using: {self.enhancer}")
        self.br = CvBridge()
        
    def enhancement_callback(self, msg):
        # ROS2 -> OpenCV
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        try:
            enhanced_image = self.enhancer.enhance(cv_image)
        except cv2error as e:
            self.get_logger().error(
                f"Error during image enhancement: {e}.\
                Publishing original image to {self.output_topic}."
            )
            enhanced_msg = msg  # Fallback to original message on error
        # OpenCV -> ROS2
        enhanced_msg = self.br.cv2_to_imgmsg(enhanced_image, encoding="bgr8")
		# Publish enhanced image
        self.publisher.publish(enhanced_msg)
