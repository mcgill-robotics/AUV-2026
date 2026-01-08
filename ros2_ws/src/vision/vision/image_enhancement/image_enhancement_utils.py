from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import cv2
import numpy as np

from vision.image_enhancement import enhancement_algorithms as enhance

class EnhanceNode(Node):
    def __init__(self,node_name:str,input_topic:str,output_topic:str,enhancer:enhance.ImageEnhancer):
        super().__init__(node_name)
        
        self.declare_parameter("input_topic", input_topic)
        self.declare_parameter("output_topic", output_topic)
        self.declare_parameter("sim", False)

        # Get parameters, include fallback to provided arguments
        self.input_topic = self.get_parameter("input_topic").value
        if not self.input_topic:
            self.get_logger().warn(
                f"No input_topic parameter provided, using default: {input_topic}"
            )
            self.input_topic = input_topic
        self.output_topic = self.get_parameter("output_topic").value
        if not self.output_topic:
            self.get_logger().warn(
                f"No output_topic parameter provided, using default: {output_topic}"
            )
            self.output_topic = output_topic
            
        self.for_sim = self.get_parameter("sim").value
        image_format = Image
        if self.for_sim:
            image_format = CompressedImage
            self.get_logger().warn(
                ("WARNING: EnhanceNode running in simulation mode:"
                 " input topic is assumed to be in compressed image format,"
                 " output topic will also be compressed.")
            )
        self.subscription = self.create_subscription(
			image_format, # Image message type
			self.input_topic, # Topic name
			self.enhancement_callback, # Callback, called on message received
			10 # QoS: if received messages > this #, start dropping oldest received ones
		)
        self.publisher = self.create_publisher(image_format, self.output_topic, 10)
        self.get_logger().info(
            (f"EnhanceNode initialized with "
            f"input topic: {self.input_topic} and "
            f"output topic: {self.output_topic}")
        )
        self.enhancer = enhancer
        self.get_logger().info(f"Using: {self.enhancer}")
        self.br = CvBridge()
        self.image_encoding = "bgr8"
        
    def enhancement_callback(self, msg):
        if (self.image_encoding!=msg.format):
            self.get_logger().info(
                (f"Setting encoding to {msg.format} from message.")
            )
            self.image_encoding = msg.format
        try:
            enhanced_msg = self.apply_enhancer(msg)
        except (cv2.error, FloatingPointError) as e:
            self.get_logger().error(
                (f"Error during image enhancement: {e}."
                f" Publishing original image to {self.output_topic}.")
            )
            enhanced_msg = msg  # Fallback to original message on error
		# Publish enhanced image (or fallback)
        self.publisher.publish(enhanced_msg)
    # may raise cv2.error or FloatingPointError
    def apply_enhancer(self, msg) -> Image | CompressedImage:
        if self.for_sim:
            # ROS2 -> OpenCV
            cv_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # raise any floating point errors as exceptions instead of warnings
            with np.errstate(invalid='raise'):
                enhanced_image = self.enhancer.enhance(cv_image)
            # OpenCV -> ROS2
            enhanced_msg = self.br.cv2_to_compressed_imgmsg(enhanced_image)
        else:
            # ROS2 -> OpenCV
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # raise any floating point errors as exceptions instead of warnings
            with np.errstate(invalid='raise'):
                enhanced_image = self.enhancer.enhance(cv_image)
            # OpenCV -> ROS2
            enhanced_msg = self.br.cv2_to_imgmsg(enhanced_image)
        return enhanced_msg