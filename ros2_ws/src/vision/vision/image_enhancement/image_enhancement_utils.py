from rclpy.node import Node, Parameter
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional
import threading

from vision.image_enhancement import enhancement_algorithms as enhance
from vision.image_enhancement import enhancement_algorithms_GPU as GPUenhance

class ImageEnhancement():
    def __init__(
        self,
        node: Node,
        enhancer: enhance.ImageEnhancer | GPUenhance.GPUImageEnhancer,
    ):
        # for now only GPU algorithms require depth
        # TODO: separate GPU logic in enhancement_algorithms_GPU from requiring depth
        
        self.node = node
        self.requires_depth = isinstance(enhancer, GPUenhance.GPUImageEnhancer)
        if self.requires_depth:
            self.node.get_logger().info("Enhancer requires depth input.")
        self.node.declare_parameter("input_topic",Parameter.Type.STRING)
        self.node.declare_parameter("output_topic",Parameter.Type.STRING)
        if self.requires_depth:
            self.node.declare_parameter("depth_topic", Parameter.Type.STRING)
        else:
            self.node.declare_parameter("depth_topic", Parameter.Type.STRING, None)  # Declare but allow None for non-depth algorithms
        self.node.declare_parameter("publish_rate_hz", Parameter.Type.INTEGER)
        self.node.declare_parameter("sim", Parameter.Type.BOOL)
        
        self.input_topic: str = self.node.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic: str = self.node.get_parameter("output_topic").get_parameter_value().string_value
        if self.requires_depth:
            self.node.get_logger().warn(
                "EnhanceNode initialized with a GPU enhancer that requires depth. Make sure to provide a valid depth topic."
            )
            self.depth_topic = self.node.get_parameter("depth_topic").get_parameter_value().string_value
            if not self.depth_topic:
                self.node.get_logger().error("Depth topic is required for this enhancer but was not provided. Please set the 'depth_topic' parameter.")
                self.node.get_logger().fatal("Shutting down EnhanceNode due to missing depth topic.")
                raise RuntimeError("Depth topic is required for this enhancer but was not provided.")
        else:
            self.depth_topic = None
        self.publish_rate_hz: int = self.node.get_parameter("publish_rate_hz").get_parameter_value().integer_value    
        self.for_sim: bool = self.node.get_parameter("sim").get_parameter_value().bool_value
        
        image_format = Image
        if self.for_sim:
            image_format = CompressedImage
            self.node.get_logger().warn(
                "EnhanceNode running in simulation mode: using compressed image format."
            )
        
        # Add locks to that callbacks can run independently of timer and not block each other. The timer will just use the latest available data when it runs.
        self._lock = threading.Lock()
        # buffer for latest image and depth
        self._latest_image: Optional[np.ndarray] = None
        # depth is optional, so it can be None if not provided or if the depth callback hasn't received data yet
        self._latest_depth: Optional[np.ndarray] = None
        # latest image  header for timestamping output
        self._latest_header = None
        # flag to indicate if a new image has been received since the last enhancement, so we don't run enhancement unnecessarily if no new data is available
        self._new_image_available = False
        
        # Image subscription
        self.image_sub = self.node.create_subscription(
            image_format,
            self.input_topic,
            self._image_callback,
            10
        )
        self.node.get_logger().info(f"Subscribed to image topic: {self.input_topic}")
        
        # Depth subscription (optional)
        if self.depth_topic:
            self.depth_sub = self.node.create_subscription(
                image_format,
                self.depth_topic,
                self._depth_callback,
                10
            )
            self.node.get_logger().info(f"Subscribed to depth topic: {self.depth_topic}")
        else:
            self.depth_sub = None
            
        self.publisher = self.node.create_publisher(image_format, self.output_topic, 10)
        
        # Timer-driven enhancement
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.node.create_timer(timer_period, self._timer_callback)
        
        self.enhancer = enhancer
        self.br = CvBridge()
        
        self.node.get_logger().info(
            f"EnhanceNode initialized: {self.input_topic} + {self.depth_topic} -> {self.output_topic} @ {self.publish_rate_hz}Hz"
        )
        self.node.get_logger().info(f"Using: {self.enhancer}")

    def _image_callback(self, msg):
        self.node.get_logger().debug("Received new image message.", once=True)  # Log once when new images start coming in
        try:
            if self.for_sim:
                cv_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            with self._lock:
                self._latest_image = cv_image
                self._latest_header = msg.header
                self._new_image_available = True
        except cv2.error as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")

    def _depth_callback(self, msg):
        self.node.get_logger().debug("Received new depth message.", once=True)  # Log once when new depth data starts coming in
        try:
            if self.for_sim:
                # passthrough preserves original encoding (32FC1, 16UC1, etc.)
                depth_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
            else:
                depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            with self._lock:
                self._latest_depth = depth_image.astype(np.float32)
        except cv2.error as e:
            self.node.get_logger().error(f"Failed to convert depth: {e}")

    def _timer_callback(self):
        with self._lock:
            if not self._new_image_available or self._latest_image is None:
                self.node.get_logger().debug("No new image available for enhancement. Skipping this cycle.", throttle_duration_sec=1)
                return  # No new image to process
            
            if self.requires_depth and self._latest_depth is None:
                self.node.get_logger().warn("Enhancer requires depth but no depth data available yet. Skipping enhancement.",throttle_duration_sec=1)
                return  # Depth required but not available, skip enhancement for now
            
            # Grab copies to release lock quickly
            image = self._latest_image
            depth = self._latest_depth
            header = self._latest_header
            self._new_image_available = False
        
        try:
            self.node.get_logger().info("Running enhancement algorithm...",once=True)  # Log once when enhancement starts
            with np.errstate(invalid='raise'):
                enhanced_image = self.enhancer.enhance(image, depth)
            
            if self.for_sim:
                enhanced_msg = self.br.cv2_to_compressed_imgmsg(enhanced_image)
            else:
                enhanced_msg = self.br.cv2_to_imgmsg(enhanced_image, encoding="bgr8")
            
            enhanced_msg.header = header
            self.publisher.publish(enhanced_msg)
            
        except (cv2.error, FloatingPointError) as e:
            self.node.get_logger().error(f"Enhancement failed: {e}. Falling back to original image.")
            # Publish original image if enhancement fails
            if self.for_sim:
                fallback_msg = self.br.cv2_to_compressed_imgmsg(image)
            else:
                fallback_msg = self.br.cv2_to_imgmsg(image, encoding="bgr8")
            fallback_msg.header = header
            self.publisher.publish(fallback_msg)
            