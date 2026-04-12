import cv2
import supervision as sv
from inference_models import AutoModel
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


def load_model(model_path: str, logger):
    """Load an inference-models AutoModel with TensorRT/CUDA acceleration."""
    logger.info(f"Loading model package from: {model_path} with TensorRT...")
    try:
        model = AutoModel.from_pretrained(
            model_path,
            onnx_execution_providers=["TensorrtExecutionProvider", "CUDAExecutionProvider"],
            default_onnx_trt_options=False
        )
        logger.info("Model successfully loaded and optimized for inference (TensorRT/CUDA).")
        return model
    except Exception as e:
        logger.error(f"Failed to load model: {e}")
        logger.fatal("Exiting due to model load failure.")
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

def publish_annotated_image_util(detector_node, img, tracked_detections):
    if not detector_node.publish_annotated_image or tracked_detections is None:
        return

    annotated = img.copy()

    labels = [f"{detector_node.class_names[int(tracked_detections.class_id[i])]} {tracked_detections.confidence[i]:.2f}"
                for i in range(len(tracked_detections)) if int(tracked_detections.class_id[i]) < len(detector_node.class_names)]

    box_annotator = sv.BoxAnnotator(thickness=2)
    label_annotator = sv.LabelAnnotator(text_thickness=2, text_scale=0.8)

    annotated = box_annotator.annotate(scene=annotated, detections=tracked_detections)
    annotated = label_annotator.annotate(scene=annotated, detections=tracked_detections, labels=labels)

    try:
        if detector_node.compressed:
            ann_msg = detector_node.bridge.cv2_to_compressed_imgmsg(annotated)
        else:
            ann_msg = detector_node.bridge.cv2_to_imgmsg(annotated, "bgr8")
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

