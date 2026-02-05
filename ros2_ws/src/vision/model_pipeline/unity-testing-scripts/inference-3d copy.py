#!/usr/bin/env python3
"""
AUV 3D Vision with Quality-Weighted Mapping
- YOLO + ZED SDK integration
- Quality score based on distance + confidence  
- Quality Lock prevents distant ghosts from corrupting good data
"""

import pyzed.sl as sl
from ultralytics import YOLO
import numpy as np
import time
import cv2
from pathlib import Path

# ============================================================================
# CONFIGURATION
# ============================================================================

SCRIPT_DIR = Path(__file__).parent.resolve()
BASE_DIR = SCRIPT_DIR.parent

MODEL_PATH = BASE_DIR / "runs/detect/yolov11s_sim_dataset7/weights/best.pt"
CONFIDENCE_THRESHOLD = 0.5
MAX_DETECTION_RANGE = 10.0  # Meters - detections beyond this have 0 distance score

# Merge thresholds
BASE_MERGE_THRESHOLD = 2.0    
MERGE_THRESHOLD_SCALE = 0.2

# Confirmation
CONSECUTIVE_FRAMES_REQUIRED = 12

# Quality thresholds
MIN_QUALITY_TO_ENTER = 0.25   # Noise rejection - below this, ignore completely
QUALITY_LOCK_THRESHOLD = 0.4  # Below this, don't update confirmed high-quality objects

# Class limits
MAX_PER_CLASS = {
    "gate": 1, "lane_marker": 1, "board": 1, "octagon": 1,
    "table": 1, "bin": 1, "shark": 2, "sawfish": 2,
    "white_pipe": 6, "red_pipe": 3
}

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
    init_params.set_from_stream("127.0.0.1", 30000)  # Comment out for physical camera
    init_params.depth_maximum_distance = MAX_DETECTION_RANGE
    zed = sl.Camera()
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Error opening ZED: {err}")
        exit(1)
    
    pos_param = sl.PositionalTrackingParameters()
    pos_param.enable_imu_fusion = True
    pos_param.enable_area_memory = True 
    pos_param.set_floor_as_origin = False 
    zed.enable_positional_tracking(pos_param)
    
    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    obj_param.max_range = MAX_DETECTION_RANGE
    zed.enable_object_detection(obj_param)
    
    return zed

# ============================================================================
# QUALITY & MAPPING LOGIC
# ============================================================================

global_map = {} 
global_id_counter = 0
current_frame = 0

def get_merge_threshold(distance: float) -> float:
    return BASE_MERGE_THRESHOLD + (distance * MERGE_THRESHOLD_SCALE)


def calculate_quality_score(confidence: float, position: np.ndarray) -> float:
    """
    Quality score 0.0-1.0 based on distance (60%) and confidence (40%).
    Close + high confidence = high quality.
    """
    dist = np.linalg.norm(position)
    
    # Distance score: 1.0 at 0m, 0.0 at MAX_DETECTION_RANGE
    dist_score = max(0.0, 1.0 - (dist / MAX_DETECTION_RANGE))
    
    # Confidence score: ZED returns 0-100, normalize to 0-1
    conf_score = min(confidence / 100.0, 1.0)
    
    # Weighted average (prioritize distance for underwater)
    return (0.6 * dist_score) + (0.4 * conf_score)


def update_global_map(detected_objects):
    global global_id_counter, global_map, current_frame
    current_frame += 1
    seen_this_frame = set()
    
    for obj in detected_objects:
        # Only use objects with valid tracking
        if obj.tracking_state != sl.OBJECT_TRACKING_STATE.OK:
            continue
            
        pos = np.array(obj.position)
        if np.isnan(pos).any() or np.isinf(pos).any():
            continue
        
        # Calculate quality score
        quality = calculate_quality_score(obj.confidence, pos)
        
        # FILTER 1: Noise rejection - low quality = ignore
        if quality < MIN_QUALITY_TO_ENTER:
            continue

        label = CLASS_NAMES[obj.raw_label] if obj.raw_label < len(CLASS_NAMES) else f"Class {obj.raw_label}"
        
        # Class limit logic
        max_allowed = MAX_PER_CLASS.get(label, 99)
        existing_ids = [gid for gid, data in global_map.items() if data['label'] == label]
        
        match_id = None
        
        # Singleton: always merge into existing
        if max_allowed == 1 and len(existing_ids) > 0:
            match_id = existing_ids[0]
        # Multi-instance: distance matching
        else:
            dist_from_cam = np.linalg.norm(pos)
            merge_thresh = get_merge_threshold(dist_from_cam)
            best_dist = float('inf')
            for gid in existing_ids:
                data = global_map[gid]
                dist = np.linalg.norm(data['position'] - pos)
                if dist < merge_thresh and dist < best_dist:
                    match_id = gid
                    best_dist = dist

        # At class limit and no match? Ignore to prevent ghosts
        if match_id is None and len(existing_ids) >= max_allowed:
            continue

        # --- UPDATE OR CREATE ---
        if match_id:
            data = global_map[match_id]
            
            # FILTER 2: Quality Lock
            # If confirmed with high quality, ignore LOW quality updates
            # But always accept HIGH quality updates (close + confident)
            if data['confirmed'] and data['max_quality'] > 0.7 and quality < QUALITY_LOCK_THRESHOLD:
                # For singletons, still update position but mark as seen
                if max_allowed > 1:
                    continue

            seen_this_frame.add(match_id)
            
            # Consecutive frame tracking
            if data['last_frame'] == current_frame - 1:
                data['consecutive'] += 1
            else:
                data['consecutive'] = 1 
            
            # DYNAMIC ALPHA: High quality = fast update, low quality = slow update
            alpha = max(0.05, min(0.5, quality))
            
            data['position'] = (1 - alpha) * data['position'] + alpha * pos
            data['last_frame'] = current_frame
            data['total_count'] += 1
            
            # Track best quality we've seen
            if quality > data['max_quality']:
                data['max_quality'] = quality
            
            if data['consecutive'] >= CONSECUTIVE_FRAMES_REQUIRED:
                data['confirmed'] = True
        else:
            # Create new object
            global_id_counter += 1
            seen_this_frame.add(global_id_counter)
            global_map[global_id_counter] = {
                'label': label,
                'position': pos.copy(),
                'consecutive': 1,
                'confirmed': False,
                'last_frame': current_frame,
                'total_count': 1,
                'max_quality': quality
            }
    
    # Prune tentative objects not seen this frame
    ids_to_remove = [
        g_id for g_id, data in global_map.items()
        if not data['confirmed'] and g_id not in seen_this_frame
    ]
    for g_id in ids_to_remove:
        del global_map[g_id]


def print_global_map(cam_pos):
    print("\033[2J\033[H", end="") 
    print(f"{'='*85}")
    print(f"  AUV VISION - Quality-Weighted Global Map")
    print(f"{'='*85}")
    print(f"  Sub Position (VIO): [{cam_pos[0]:>7.2f}, {cam_pos[1]:>7.2f}, {cam_pos[2]:>7.2f}]")
    
    confirmed = {k: v for k, v in global_map.items() if v['confirmed']}
    tentative = {k: v for k, v in global_map.items() if not v['confirmed']}
    
    if not global_map:
        print("  No objects detected.")
    else:
        if confirmed:
            print(f"\n  ✓ CONFIRMED ({len(confirmed)})")
            print(f"  {'ID':<4} {'Label':<15} {'X':>7} {'Y':>7} {'Z':>7} {'Qual':>5} {'Seen':>6}")
            print(f"  {'-'*4} {'-'*15} {'-'*7} {'-'*7} {'-'*7} {'-'*5} {'-'*6}")
            for g_id, data in sorted(confirmed.items(), key=lambda x: x[1]['label']):
                pos = data['position']
                print(f"  {g_id:<4} {data['label']:<15} {pos[0]:>7.2f} {pos[1]:>7.2f} {pos[2]:>7.2f} {data['max_quality']:>5.2f} {data['total_count']:>6}")
        
        if tentative:
            print(f"\n  ? TENTATIVE ({len(tentative)})")
            print(f"  {'ID':<4} {'Label':<15} {'Consec':>8}")
            for g_id, data in sorted(tentative.items()):
                print(f"  {g_id:<4} {data['label']:<15} {data['consecutive']:>8}/{CONSECUTIVE_FRAMES_REQUIRED}")
    
    print(f"{'='*85}")

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
    last_valid_cam_pos = [0.0, 0.0, 0.0]
    
    print("Starting... Press 'q' to quit.")
    
    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            image_bgr = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)
            
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
            
            zed.ingest_custom_box_objects(custom_boxes)
            
            # Only update map when VIO is OK
            state = zed.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
            
            if state == sl.POSITIONAL_TRACKING_STATE.OK:
                last_valid_cam_pos = cam_pose.get_translation().get()
                
                objects = sl.Objects()
                zed.retrieve_objects(objects, obj_runtime_param)
                update_global_map(objects.object_list)
                
                tracking_status = "VIO: OK"
            else:
                tracking_status = "VIO: LOST - MAPPING PAUSED"
            
            print_global_map(last_valid_cam_pos)
            print(f"  STATUS: {tracking_status}")

            annotated_frame = results[0].plot()
            color = (0, 255, 0) if state == sl.POSITIONAL_TRACKING_STATE.OK else (0, 0, 255)
            cv2.putText(annotated_frame, tracking_status, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            cv2.imshow("AUV Vision", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cv2.destroyAllWindows()
    zed.close()

if __name__ == "__main__":
    main()