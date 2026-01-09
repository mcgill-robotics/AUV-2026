#!/usr/bin/env python3
"""
Basic 3D Inference - YOLO + ZED SDK
Just shows raw 3D object output from ZED without filtering/mapping.
"""

import pyzed.sl as sl
from ultralytics import YOLO
import numpy as np
import cv2
from pathlib import Path

# ============================================================================
# CONFIGURATION
# ============================================================================

SCRIPT_DIR = Path(__file__).parent.resolve()
BASE_DIR = SCRIPT_DIR.parent

MODEL_PATH = BASE_DIR / "runs/detect/yolov11s_sim_dataset7/weights/best.pt"
CONFIDENCE_THRESHOLD = 0.5
MAX_DETECTION_RANGE = 15.0

CLASS_NAMES = [
    "gate", "lane_marker", "red_pipe", "white_pipe", "octagon",
    "table", "bin", "board", "shark", "sawfish"
]

# ============================================================================
# ZED INITIALIZATION
# ============================================================================

def init_zed():
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.SVGA
    init_params.camera_fps = 30
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.depth_maximum_distance = MAX_DETECTION_RANGE
    init_params.set_from_stream("127.0.0.1", 30000)  # Comment out for physical camera

    zed = sl.Camera()
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Error opening ZED: {err}")
        exit(1)
    
    # Enable positional tracking
    pos_param = sl.PositionalTrackingParameters()
    pos_param.enable_imu_fusion = True
    pos_param.enable_area_memory = True
    pos_param.set_floor_as_origin = False
    zed.enable_positional_tracking(pos_param)
    
    # Enable object detection with custom boxes
    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    obj_param.max_range = MAX_DETECTION_RANGE
    zed.enable_object_detection(obj_param)
    
    return zed

# ============================================================================
# MAIN
# ============================================================================

def main():
    model = YOLO(str(MODEL_PATH))
    zed = init_zed()
    
    runtime_params = sl.RuntimeParameters()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 35
    
    image = sl.Mat()
    cam_pose = sl.Pose()
    
    print("Starting basic 3D inference... Press 'q' to quit.")
    
    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Get image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            image_bgr = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)
            
            # YOLO inference
            results = model.predict(image_bgr, verbose=False, conf=CONFIDENCE_THRESHOLD)
            
            # Prepare custom boxes for ZED
            custom_boxes = []
            for r in results:
                for box in r.boxes:
                    tmp = sl.CustomBoxObjectData()
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    tmp.bounding_box_2d = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
                    tmp.label = int(box.cls)
                    tmp.probability = float(box.conf)
                    tmp.is_static = True
                    custom_boxes.append(tmp)
            
            # Feed boxes to ZED
            zed.ingest_custom_box_objects(custom_boxes)
            
            # Retrieve 3D objects
            objects = sl.Objects()
            zed.retrieve_objects(objects, obj_runtime_param)
            
            # Get camera position
            state = zed.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
            cam_pos = cam_pose.get_translation().get() if state == sl.POSITIONAL_TRACKING_STATE.OK else [0, 0, 0]
            
            # Print output
            print("\033[2J\033[H", end="")  # Clear
            print(f"{'='*80}")
            print(f"  BASIC 3D INFERENCE")
            print(f"{'='*80}")
            print(f"  Camera Position: [{cam_pos[0]:>7.2f}, {cam_pos[1]:>7.2f}, {cam_pos[2]:>7.2f}]")
            print(f"  VIO Status: {state}")
            print(f"  Objects Detected: {len(objects.object_list)}")
            print(f"{'='*80}")
            
            if objects.object_list:
                print(f"  {'ID':<6} {'Label':<15} {'X':>7} {'Y':>7} {'Z':>7} {'Conf':>5} {'Track State':<15}")
                print(f"  {'-'*6} {'-'*15} {'-'*7} {'-'*7} {'-'*7} {'-'*5} {'-'*15}")
                for obj in objects.object_list:
                    pos = obj.position
                    label = CLASS_NAMES[obj.raw_label] if obj.raw_label < len(CLASS_NAMES) else f"Class {obj.raw_label}"
                    print(f"  {obj.id:<6} {label:<15} {pos[0]:>7.2f} {pos[1]:>7.2f} {pos[2]:>7.2f} {obj.confidence:>5.0f} {obj.tracking_state}")
            else:
                print("  No objects detected.")
            
            print(f"{'='*80}")
            
            # Display
            annotated_frame = results[0].plot()
            cv2.imshow("Basic 3D Inference", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cv2.destroyAllWindows()
    zed.close()

if __name__ == "__main__":
    main()