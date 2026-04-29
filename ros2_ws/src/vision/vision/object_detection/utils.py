from pathlib import Path

import os
import cv2
import numpy as np
import supervision as sv
from inference_models import AutoModel, BackendType
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import euler_matrix, quaternion_from_matrix

# Enable CUDA Graphs globally (Native TRT only, ignored by ONNX)
os.environ["ENABLE_AUTO_CUDA_GRAPHS_FOR_TRT_BACKEND"] = "True"


def load_model(model_path: str, logger):
    """
    Unified loader for local model packages.
    Automatically applies TensorRT optimizations for ONNX and 
    CUDA Graphs for Native TRT based on model_config.json.
    """
    logger.info(f"Initializing model package from: {model_path}")

    # 1. Define our high-performance ONNX settings
    # These are ignored if the model_config.json specifies backend_type: "trt"
    trt_ep = ("TensorrtExecutionProvider", {
        "trt_engine_cache_enable": True,
        "trt_engine_cache_path": model_path,
        "trt_fp16_enable": True 
    })

    try:
        # 2. Pass everything to the AutoModel loader
        # It will use what it needs and discard the rest.
        model = AutoModel.from_pretrained(
            model_path,
            onnx_execution_providers=[trt_ep, "CUDAExecutionProvider"],
            default_onnx_trt_options=False
        )
        
        # 3. Dynamic Warmup Check: 
        # Only Native TRT models benefit from a warmup to capture the CUDA Graph.
        # Check the internal backend type of the loaded model.
        if hasattr(model, 'backend') and model.backend == BackendType.TRT:
            logger.info("Native TRT detected: Warming up and capturing CUDA Graph...")
            warmup_image = np.zeros((640, 640, 3), dtype=np.uint8)
            model(warmup_image)
            
        logger.info("Model load complete.")
        return model
        
    except Exception as e:
        logger.error(f"Failed to load model package: {e}")
        raise


def get_detections(detector_node, img):
    try:
        # Run inference using the unified inference-models API
        predictions = detector_node.model(img, confidence=detector_node.conf_threshold)

        if not predictions:
            return None

        # Extract the first batch element and convert to supervision format
        tracked_detections = predictions[0].to_supervision()

        # Return None if no objects passed the confidence threshold
        if len(tracked_detections) == 0:
            return None

        return tracked_detections
    except Exception as e:
        detector_node.node.get_logger().error(f"Inference failed: {e}")
        return None

def publish_annotated_image_util(detector_node, img, tracked_detections, stamp, frame_id):
    if not detector_node.publish_annotated_image:
        return

    output_image = img.copy()

    # Draw annotations only when enabled and detections are available.
    if (
        getattr(detector_node, "annotated_image_enabled", True)
        and tracked_detections is not None
        and len(tracked_detections) > 0
    ):
        labels = [f"{detector_node.class_names[int(tracked_detections.class_id[i])]} {tracked_detections.confidence[i]:.2f}"
                    for i in range(len(tracked_detections)) if int(tracked_detections.class_id[i]) < len(detector_node.class_names)]

        box_annotator = sv.BoxAnnotator(thickness=2)
        label_annotator = sv.LabelAnnotator(text_thickness=2, text_scale=0.8)

        output_image = box_annotator.annotate(scene=output_image, detections=tracked_detections)
        output_image = label_annotator.annotate(scene=output_image, detections=tracked_detections, labels=labels)

    try:
        if detector_node.compressed:
            ann_msg = detector_node.bridge.cv2_to_compressed_imgmsg(output_image)
        else:
            ann_msg = detector_node.bridge.cv2_to_imgmsg(output_image, "bgr8")
        
        ann_msg.header.stamp = stamp
        ann_msg.header.frame_id = frame_id
        detector_node.pub_annotated_image.publish(ann_msg)
    except Exception as e:
        detector_node.node.get_logger().error(f"Failed to publish annotated image: {e}")

def build_detection2d_msg(detector_node, tracked_detections):
    det_msg = Detection2DArray()
    if tracked_detections is None: return det_msg
    
    det_objects = []
    for i in range(len(tracked_detections)):
        x1, y1, x2, y2 = tracked_detections.xyxy[i]
        cx = float((x1 + x2) / 2)
        cy = float((y1 + y2) / 2)
        w = float(x2 - x1)
        h = float(y2 - y1)
        conf = float(tracked_detections.confidence[i])
        cls_id = int(tracked_detections.class_id[i])

        if cls_id >= len(detector_node.class_names):
            continue

        label = detector_node.class_names[cls_id]

        detection = Detection2D()
        detection.bbox.center.position.x = cx
        detection.bbox.center.position.y = cy
        detection.bbox.size_x = w
        detection.bbox.size_y = h

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
        hypothesis.hypothesis.score = conf
        
        detection.results = [hypothesis]
        det_objects.append(detection)

    det_msg.detections = det_objects
    return det_msg

# --- Shared Dataset Service Logic ---
def toggle_collection_callback_util(detector_node, request, response):
    requested_interval = getattr(request, "time_interval", detector_node.collection_interval)
    if requested_interval <= 0.0:
        response.success = False
        response.message = "Collection interval must be > 0 seconds"
        detector_node.node.get_logger().warn(response.message)
        return response

    detector_node._collecting = request.data
    detector_node.collection_interval = float(requested_interval)
    if detector_node._collecting:
        Path(detector_node.collection_dir).mkdir(parents=True, exist_ok=True)
        if hasattr(detector_node, 'depth_collection_dir'):
            Path(detector_node.depth_collection_dir).mkdir(parents=True, exist_ok=True)
            detector_node.node.get_logger().info(
                f"Image collection ENABLED every {detector_node.collection_interval:.2f}s "
                f"→ {detector_node.collection_dir} and {detector_node.depth_collection_dir}"
            )
        else:
            detector_node.node.get_logger().info(
                f"Image collection ENABLED every {detector_node.collection_interval:.2f}s "
                f"→ {detector_node.collection_dir}"
            )
    else:
        detector_node.node.get_logger().info(
            f"Image collection DISABLED (interval preserved at {detector_node.collection_interval:.2f}s)"
        )
    response.success = True
    response.message = (
        f"Collection {'enabled' if detector_node._collecting else 'disabled'} "
        f"with interval {detector_node.collection_interval:.2f}s"
    )
    return response

# --- Shared Object Spatial Math Utilities ---
def get_vector3_parameter(node, name: str) -> np.ndarray:
    values = node.get_parameter(name).get_parameter_value().double_array_value
    if len(values) != 3:
        raise ValueError(f"Parameter '{name}' must contain exactly 3 values.")
    return np.array([float(values[0]), float(values[1]), float(values[2])], dtype=float)

def build_transform_matrix_from_xyz_rpy(
    translation_xyz: np.ndarray,
    rotation_rpy: np.ndarray,
) -> np.ndarray:
    transform = euler_matrix(
        float(rotation_rpy[0]),
        float(rotation_rpy[1]),
        float(rotation_rpy[2]),
    )
    transform[0:3, 3] = translation_xyz
    return transform

def build_quaternion_msg(orientation_xyzw: np.ndarray) -> Quaternion:
    orientation = Quaternion()
    orientation.x = float(orientation_xyzw[0])
    orientation.y = float(orientation_xyzw[1])
    orientation.z = float(orientation_xyzw[2])
    orientation.w = float(orientation_xyzw[3])
    return orientation

def build_pose_message(
    translation: np.ndarray,
    orientation_xyzw: np.ndarray,
    frame_id: str,
    stamp,
) -> PoseStamped:
    pose_msg = PoseStamped()
    pose_msg.header.stamp = stamp
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.position.x = float(translation[0])
    pose_msg.pose.position.y = float(translation[1])
    pose_msg.pose.position.z = float(translation[2])
    pose_msg.pose.orientation = build_quaternion_msg(orientation_xyzw)
    return pose_msg

def build_pose_message_from_transform(
    transform: np.ndarray,
    frame_id: str,
    stamp,
) -> PoseStamped:
    translation = transform[0:3, 3]
    orientation_xyzw = quaternion_from_matrix(transform)
    return build_pose_message(translation, orientation_xyzw, frame_id, stamp)

def bbox_from_xyxy(x1: float, y1: float, x2: float, y2: float):
    size_x = max(0.0, float(x2) - float(x1))
    size_y = max(0.0, float(y2) - float(y1))
    center_x = float(x1) + size_x / 2.0
    center_y = float(y1) + size_y / 2.0
    return center_x, center_y, size_x, size_y

def bbox_from_zed_corners(corners) -> tuple[float, float, float, float]:
    if corners is None or len(corners) == 0:
        return 0.0, 0.0, 0.0, 0.0

    xs = [float(point[0]) for point in corners]
    ys = [float(point[1]) for point in corners]
    return bbox_from_xyxy(min(xs), min(ys), max(xs), max(ys))
