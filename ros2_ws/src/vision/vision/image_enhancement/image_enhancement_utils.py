from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
from cv_bridge.core import CvBridgeError
import cv2
import numpy as np

from vision.image_enhancement import enhancement_algorithms as enhance

class EnhanceNode(Node):
    def __init__(self,node_name:str,enhancer:enhance.ImageEnhancer):
        super().__init__(node_name)
        
        self.declare_parameter("input_topic", Parameter.Type.STRING)
        self.declare_parameter("output_topic", Parameter.Type.STRING)
        self.declare_parameter("compressed", Parameter.Type.BOOL)
        self.declare_parameter("queue_size", Parameter.Type.INTEGER)

        self.input_topic:str = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic:str = self.get_parameter("output_topic").get_parameter_value().string_value
        self.compressed:bool = self.get_parameter("compressed").get_parameter_value().bool_value
        self.queue_size:int = self.get_parameter("queue_size").get_parameter_value().integer_value
        
        input_format = Image
        if self.compressed:
            input_format = CompressedImage
            self.get_logger().warn(
                ("WARNING: EnhanceNode running in compressed mode."
                 " Input and output images will be compressed.")
            )
        self.subscription = self.create_subscription(
			input_format, # Image message type
			self.input_topic, # Topic name
			self.enhancement_callback, # Callback, called on message received
			self.queue_size # QoS: if received messages > this #, start dropping oldest received ones
		)
        self.publisher = self.create_publisher(input_format, self.output_topic, self.queue_size)
        self.get_logger().info(
            (f"EnhanceNode initialized with "
            f"input topic: {self.input_topic} and "
            f"output topic: {self.output_topic}")
        )
        self.enhancer = enhancer
        self.get_logger().info(f"Using: {self.enhancer}")
        self.br = CvBridge()
        
    def enhancement_callback(self, msg):
        try:
            enhanced_msg = self.apply_enhancer(msg)
        except (cv2.error, FloatingPointError, CvBridgeError) as e:
            self.get_logger().error(
                (f"Error during image enhancement: {e}."
                f" Publishing original image to {self.output_topic}.")
            )
            enhanced_msg = msg  # Fallback to original message on error
        # ensure header timestamps and frame_ids are preserved
        enhanced_msg.header = msg.header
		# Publish enhanced image (or fallback)
        self.publisher.publish(enhanced_msg)
    # may raise cv2.error or FloatingPointError
    def apply_enhancer(self, msg) -> Image | CompressedImage:
        if self.compressed:
            # ROS2 -> OpenCV
            cv_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # raise any floating point errors as exceptions instead of warnings
            with np.errstate(invalid='raise'):
                enhanced_image = self.enhancer.enhance(cv_image)
            # OpenCV -> ROS2
            if "png" in msg.format:
                dst = ".png"
            else:
                dst = ".jpg"  # default jpeg, no clean way to do input = output format for compressed
            enhanced_msg = self.br.cv2_to_compressed_imgmsg(enhanced_image, dst_format=dst)
            enhanced_msg.format = msg.format
        else:
            # ROS2 -> OpenCV
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # raise any floating point errors as exceptions instead of warnings
            with np.errstate(invalid='raise'):
                enhanced_image = self.enhancer.enhance(cv_image)
            # OpenCV -> ROS2
            enhanced_msg = self.br.cv2_to_imgmsg(enhanced_image, encoding=msg.encoding)
        return enhanced_msg