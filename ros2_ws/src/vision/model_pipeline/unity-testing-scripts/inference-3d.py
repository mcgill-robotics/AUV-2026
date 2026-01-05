#!/usr/bin/env python3
import pyzed.sl as sl
from ultralytics import YOLO
import numpy as np
import time
import cv2
from pathlib import Path

# BASE DIR for paths
SCRIPT_DIR = Path(__file__).parent.resolve()
BASE_DIR = SCRIPT_DIR.parent

test_time = 0  # seconds (set to 0 for infinite loop)
model_path = BASE_DIR / "runs/detect/yolov11s_sim_dataset7/weights/best.pt"
model = YOLO(str(model_path))

# ZED initialization
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.SVGA
init_params.camera_fps = 30
init_params.coordinate_units = sl.UNIT.METER
init_params.set_from_stream("127.0.0.1", 30000)  # Comment out if using physical camera

zed = sl.Camera()
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"Error opening ZED: {err}")
    exit(1)

# Enable positional tracking (required for object tracking and world coordinates)
pos_param = sl.PositionalTrackingParameters()
zed.enable_positional_tracking(pos_param)

# Enable object detection with custom box mode
obj_param = sl.ObjectDetectionParameters()
obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
obj_param.enable_tracking = True  # Enables Kalman filtering for stability/false positive reduction
zed.enable_object_detection(obj_param)

runtime_params = sl.RuntimeParameters()
obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
# obj_runtime_param.detection_confidence = 50  # Filter detections below 50%

# For image retrieval
image = sl.Mat()

# Metrics
num_images = 0
total_time = 0.0

# Custom class names from your model (edit based on your dataset)
class_names = [
    "gate",
    "lane_marker",
    "red_pipe",
    "white_pipe",
    "octagon",
    "table",
    "bin",
    "board",
    "shark",
    "sawfish"
]

print("Starting inference with 3D integration... Press 'q' to quit.")

loop_start_time = time.time()
while True:
    if test_time > 0 and (time.time() > loop_start_time + test_time):
        break

    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve left image
        zed.retrieve_image(image, sl.VIEW.LEFT)
        image_np = image.get_data()
        image_bgr = cv2.cvtColor(image_np, cv2.COLOR_BGRA2BGR)

        # Run YOLO inference (>50% conf via param)
        start_time = time.time()
        results = model.predict(image_bgr, verbose=False, conf=0.5)
        end_time = time.time()
        total_time += end_time - start_time
        num_images += 1

        # Prepare custom boxes for ingestion
        custom_boxes = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                tmp = sl.CustomBoxObjectData()
                # Bounding box corners (clockwise, in pixels)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                tmp.bounding_box_2d = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
                tmp.label = int(box.cls)  # Class ID
                tmp.probability = float(box.conf)  # Confidence
                tmp.unique_object_id = sl.generate_unique_id()  # For tracking association
                tmp.is_grounded = False  # Set True if objects are on floor plane
                custom_boxes.append(tmp)

        # Ingest custom detections into ZED SDK
        zed.ingest_custom_box_objects(custom_boxes)

        # Retrieve 3D objects
        objects = sl.Objects()
        zed.retrieve_objects(objects, obj_runtime_param)

        # Print 3D info for testing (position in meters relative to camera)
        if objects.is_new and len(objects.object_list) > 0:
            print(f"\n{'='*70}")
            print(f"  DETECTED {len(objects.object_list)} OBJECT(S)")
            print(f"{'='*70}")
            print(f"  {'Label':<15} {'ID':>4}  {'X (m)':>8} {'Y (m)':>8} {'Z (m)':>8}  {'Conf':>5}")
            print(f"  {'-'*15} {'-'*4}  {'-'*8} {'-'*8} {'-'*8}  {'-'*5}")
            for obj in objects.object_list:
                pos = obj.position  # 3D position (x, y, z)
                label = class_names[obj.raw_label] if obj.raw_label < len(class_names) else f"Class {obj.raw_label}"
                conf = obj.confidence
                print(f"  {label:<15} {obj.id:>4}  {pos[0]:>8.3f} {pos[1]:>8.3f} {pos[2]:>8.3f}  {conf:>5.1f}%")
            print(f"{'='*70}\n")

        # Draw 2D annotations and display
        annotated_frame = results[0].plot()
        cv2.imshow("AUV Vision - YOLO + ZED 3D", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Cleanup
cv2.destroyAllWindows()
zed.disable_object_detection()
zed.disable_positional_tracking()
zed.close()

print(f"Total images processed: {num_images}")
print(f"Total time spent inferring: {total_time:.2f}s")
print(f"Average inference time: {total_time / num_images * 1000:.2f} ms")