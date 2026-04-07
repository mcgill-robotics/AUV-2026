import cv2
import supervision as sv
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

def process_image(detector_node, img, stamp_time):
    det_msg = Detection2DArray()
    det_objects = []

    if detector_node.model_type == 'rfdetr':
        try:
            tracked_detections = detector_node.model.predict(img, threshold=detector_node.conf_threshold)
        except Exception as e:
            detector_node.node.get_logger().error(f"RF-DETR predict failed: {e}")
            return
    else:
        try:
            results_list = detector_node.model.predict(img, iou=0.1, agnostic_nms=False, device=detector_node.device, verbose=False)  
            if not results_list:
                return
            tracked_detections = sv.Detections.from_ultralytics(results_list[0])
        except Exception as e:
            detector_node.node.get_logger().error(f"YOLO failed: {e}")
            return

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
    detector_node.pub_detections.publish(det_msg)

    if detector_node.publish_annotated_image:
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

    current_time = detector_node.node.get_clock().now()
    time_diff = (current_time - stamp_time).nanoseconds / 1e9
    
    detector_node.node.get_logger().debug(f"Detection latency: {time_diff:.9f} s | Active detections: {len(tracked_detections)}")
