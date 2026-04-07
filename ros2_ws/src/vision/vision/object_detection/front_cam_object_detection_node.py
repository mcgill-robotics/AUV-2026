#!/usr/bin/env python3
import pyzed.sl as sl
from rclpy.node import Node
from rclpy.parameter import Parameter

import os
import cv2
from time import sleep
import supervision as sv
from rfdetr import RFDETRSmall
import torch
from cv_bridge import CvBridge
from ultralytics import YOLO
from vision.object_detection.utils import process_image

from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class FrontCamObjectDetectorNode():
    def __init__(self, node: Node):
        self.node = node
        self.node.declare_parameter('class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_path', Parameter.Type.STRING)
        self.node.declare_parameter('detection_topic', Parameter.Type.STRING)
        self.node.declare_parameter('depth_map_topic', Parameter.Type.STRING)        
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated_image', False)
        self.node.declare_parameter("compressed", Parameter.Type.BOOL)
        self.node.declare_parameter("model_type", "yolo")
        self.node.declare_parameter("confidence_threshold", 0.40)

        # we consume stream_ip and sim properties for zed sdk configuration
        self.node.declare_parameter('sim', Parameter.Type.BOOL)
        self.node.declare_parameter('stream_ip', Parameter.Type.STRING)
        self.node.declare_parameter('stream_port', Parameter.Type.INTEGER)

        self.timer = self.node.create_timer(1.0/30.0, self.run_object_detection)
 
        self.class_names = list(self.node.get_parameter('class_names').get_parameter_value().string_array_value)
        self.node.get_logger().info(f"Class names: {self.class_names}")
        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        detection_topic = self.node.get_parameter('detection_topic').get_parameter_value().string_value
        depth_map_topic = self.node.get_parameter('depth_map_topic').get_parameter_value().string_value
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value
        self.model_type = self.node.get_parameter('model_type').get_parameter_value().string_value
        self.conf_threshold = self.node.get_parameter('confidence_threshold').get_parameter_value().double_value
        sim = self.node.get_parameter('sim').get_parameter_value().bool_value
        stream_port = self.node.get_parameter('stream_port').get_parameter_value().integer_value

        input_format = CompressedImage if self.compressed else Image
        if self.compressed:
            self.node.get_logger().warn(
                (f"WARNING: {self.node.get_name()} running in compressed mode."
                 " Input image assumed to be in compressed format.")
            )
            
        self.bridge = CvBridge()
        if not os.path.exists(model_path):
            self.node.get_logger().error(f"Model path does not exist: {model_path}")
            self.node.get_logger().fatal("Exiting due to missing model.")
            raise FileNotFoundError(f"Model path does not exist: {model_path}")
        
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA if sim else sl.RESOLUTION.VGA
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.enable_image_validity_check = True
        init_params.input = sl.InputType()

        if sim:
            stream_ip = self.node.get_parameter('stream_ip').get_parameter_value().string_value
            init_params.set_from_stream(stream_ip, stream_port)

        self.image_buffer = sl.Mat()
        self.depth_buffer = sl.Mat()

        for i in range(5):
            result = self.zed.open(init_params)
            if result == sl.ERROR_CODE.SUCCESS: break
            else:
                self.node.get_logger().error(f"Failed to open zed camera, retrying {5-i} more times")
                sleep(2)
        
        if result != sl.ERROR_CODE.SUCCESS:
            self.node.get_logger().fatal("Failed to open zed camera after 5 attempts, exiting")
            exit(1)

        if not sim:
            # Configure streaming
            stream_params = sl.StreamingParameters()
            stream_params.codec = sl.STREAMING_CODEC.H264
            stream_params.port = stream_port
            stream_params.bitrate = 8000 # kbps

            # Enable streaming
            err = self.zed.enable_streaming(stream_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.node.get_logger().error(f"Streaming not enabled: {err}")
                self.zed.close()
                exit(1)

        # Avoiding CPU Oversubscription
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        
        # Load Model based on model_type
        if self.model_type == 'rfdetr':
            self.node.get_logger().info(f"Loading fine-tuned RF-DETRSmall from: {model_path} on GPU...")
            
            device = "cuda" if torch.cuda.is_available() else "cpu"
            self.model = RFDETRSmall(pretrain_weights=model_path, device=device)
                
            try:
                self.model.optimize_for_inference()
                self.node.get_logger().info("RF-DETRSmall optimized for inference (TensorRT/Torch Compile)")
            except Exception as e:
                self.node.get_logger().warn(f"Failed to optimize inference: {e}")
        else:
            self.model = YOLO(model_path)
            self.node.get_logger().info(f"Loaded YOLO model from: {model_path}")
            if torch.cuda.is_available():
                self.device = 0
                self.node.get_logger().info("Using CUDA")
            else:
                self.device = "cpu"
                self.node.get_logger().warn("Using CPU")
        
        self.node.get_logger().info(f"Setting QOL queue size to: {queue_size}")
        
        self.pub_detections = self.node.create_publisher(
            Detection2DArray,
            detection_topic,
            queue_size
        )

        self.pub_depth_map = self.node.create_publisher(
            CompressedImage if self.compressed else Image,
            depth_map_topic,
            queue_size
        )

        self.node.get_logger().info(f"Publishing to output topic: {detection_topic}")

        # Publisher for annotated debug image
        if self.publish_annotated_image:
            publish_topic = detection_topic + "/annotated" + ("/compressed" if self.compressed else "")
            self.pub_annotated_image = self.node.create_publisher(
                input_format,
                publish_topic,
                queue_size
            )
            self.node.get_logger().info(f"Publishing annotated debug image to: {publish_topic}")
        
        self.node.get_logger().info(f"{self.node.get_name()} initialized.")

    def run_object_detection(self):
        result = self.zed.grab()
        if result != sl.ERROR_CODE.SUCCESS:
            self.node.get_logger().warn("Zed.grab() failed, continuing")
            return
        
        stamp_time = self.node.get_clock().now()
        
        self.zed.retrieve_image(self.image_buffer, sl.VIEW.LEFT)

        # fully handle depth map first (better caching)
        if False:
            self.zed.retrieve_measure(self.depth_buffer, sl.MEASURE.DEPTH)
            depth = cv2.cvtColor(self.depth_buffer.get_data(), cv2.COLOR_RGBA2RGB)
            depth_map = self.bridge.cv2_to_compressed_imgmsg(depth)
            self.pub_depth_map.publish(depth_map)

        img = cv2.cvtColor(self.image_buffer.get_data(), cv2.COLOR_RGBA2RGB)
        process_image(self, img, stamp_time)
