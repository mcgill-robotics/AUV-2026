#!/usr/bin/env python3
"""
Integrated Vision Node (YOLO + ZED + Kalman Tracker)
1. Runs YOLO on ZED images.
2. Gets 3D Depth & Covariance from ZED.
3. FILTERS data using a Multi-Object Kalman Tracker (GNN + Mahalanobis).
4. Publishes ONLY stable, confirmed tracks to ROS.
"""

import rclpy
from rclpy.node import Node
import pyzed.sl as sl
from ultralytics import YOLO
import numpy as np
import cv2
from pathlib import Path
from auv_msgs.msg import VisionObject, VisionObjectArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

# Import extracted tracker
from vision.underwater_object_tracker import UnderwaterObjectTracker


# =========================================
# ROS NODE
# =========================================
class Vision3DNode(Node):
    """
    Integrated vision node: YOLO detection + ZED depth + Kalman tracking.
    Publishes only confirmed, stable tracks to /vision/objects_3d.
    """
    
    CLASS_NAMES = [
        "gate", "lane_marker", "red_pipe", "white_pipe", "octagon",
        "table", "bin", "board", "shark", "sawfish"
    ]

    def __init__(self):
        super().__init__('vision_3d_node')

        # --- Parameters ---
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('use_stream', True)
        self.declare_parameter('stream_ip', '127.0.0.1')
        self.declare_parameter('stream_port', 30000)
        self.declare_parameter('show_detections', True)  # Show YOLO bounding boxes
        self.declare_parameter('debug_logs', False)       # Toggle heavy debug table
        self.declare_parameter('min_new_track_distance', 0.5)

        model_path = self.get_parameter('model_path').value
        if not model_path:
            model_path = str(Path.home() / 'auv_ws/src/model_pipeline/runs/detect/yolov11s_sim_dataset7/weights/best.pt')
            
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.max_range = self.get_parameter('max_range').value
        self.show_detections = self.get_parameter('show_detections').value
        self.debug_logs = self.get_parameter('debug_logs').value
        min_new_track_distance = self.get_parameter('min_new_track_distance').value
        
        # --- Publishers ---
        # Publishes confirmed TRACKS, not raw detections
        self.tracks_pub = self.create_publisher(VisionObjectArray, '/vision/objects_3d', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/vio_pose', 10)

        # --- Init YOLO & ZED ---
        
        # TODO: Changes the hardware that YOLO uses based on CPU or GPU
        self.get_logger().info(f'Loading YOLO: {model_path}')
        self.model = YOLO(model_path)
        self.zed = self._init_zed()

        # --- Runtime ---
        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA 
        
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.obj_runtime_param.detection_confidence_threshold = 10
        
        self.image = sl.Mat()
        self.cam_pose = sl.Pose() 
        self.objects = sl.Objects()
        # --- Sensors ---
        self.sensor_depth = 0.0 
        self.depth_sub = self.create_subscription(
            Float64, '/sensors/depth/data', self._depth_callback, 10)
        
        # --- TRACKER ---
        self.tracker = UnderwaterObjectTracker(min_new_track_distance=min_new_track_distance)
        self.last_time = self.get_clock().now()

        # --- Timer (30 FPS) ---
        self.timer = self.create_timer(1.0 / 30.0, self._process_frame)
        self.get_logger().info('Vision 3D Node + Tracker Initialized')

    def _init_zed(self) -> sl.Camera:
        """Initialize ZED camera with tracking DISABLED (we do our own)."""
        init_params = sl.InitParameters()
        
        # SVGA corresponds to 960 * 600
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.depth_maximum_distance = self.max_range
        init_params.enable_image_validity_check = True  # Detect corrupted frames
        
        if self.get_parameter('use_stream').value:
            ip = self.get_parameter('stream_ip').value
            port = self.get_parameter('stream_port').value
            self.get_logger().info(f"Connecting to stream: {ip}:{port}")
            init_params.set_from_stream(ip, port)

        zed = sl.Camera()
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to open ZED: {err}')

        # Enable VIO for pose
        pos_param = sl.PositionalTrackingParameters()
        pos_param.enable_imu_fusion = True
        pos_param.set_floor_as_origin = False
        zed.enable_positional_tracking(pos_param)

        # Enable Object Detection (Raw Mode - no ZED tracking)
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = False  # We do our own tracking
        obj_param.max_range = self.max_range
        zed.enable_object_detection(obj_param)
        
        return zed

    def _process_frame(self):
        """Main processing loop - detect, track, publish."""
        # Calculate dt
        current_time = self.get_clock().now()
        
        # QUESTION: Does this remain numerically accurate? Or does this result in issues
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return  # Prevent division by zero on startup

        grab_status = self.zed.grab(self.runtime_params)
        if grab_status == sl.ERROR_CODE.CORRUPTED_FRAME:
            self.get_logger().warn("Corrupted frame detected - skipping frame")
            return
        if grab_status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn("Failed to grab frame - skipping frame")
            return

        # Check frame health status
        health = self.zed.get_health_status()
        if health.low_image_quality:
            self.get_logger().warn("Low image quality detected - skipping frame")
            return
        if health.low_lighting:
            self.get_logger().debug("Low lighting conditions")
        if health.low_depth_reliability:
            self.get_logger().debug("Low depth reliability")

        # 1. YOLO Inference
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        img_bgr = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)
        
        # TODO: Allow IOU parameterization via a configuration file
        results = self.model.predict(img_bgr, verbose=False,iou=0.1, agnostic_nms=True, conf=self.conf_thresh)

        # 2. Ingest detections into ZED for 3D estimation
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
                
                # TODO: Integrate maximum tracking distance
                
                # Draw detection on image for visualization
                if self.show_detections:
                    x1_int, y1_int, x2_int, y2_int = int(x1), int(y1), int(x2), int(y2)
                    label_idx = int(box.cls)
                    label = self.CLASS_NAMES[label_idx] if label_idx < len(self.CLASS_NAMES) else str(label_idx)
                    conf = float(box.conf)
                    
                    # Draw box
                    cv2.rectangle(img_bgr, (x1_int, y1_int), (x2_int, y2_int), (0, 255, 0), 2)
                    
                    # Draw label with background
                    text = f"{label}: {conf:.2f}"
                    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(img_bgr, (x1_int, y1_int - th - 4), (x1_int + tw, y1_int), (0, 255, 0), -1)
                    cv2.putText(img_bgr, text, (x1_int, y1_int - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Show visualization window
        if self.show_detections:
            cv2.imshow("YOLO Detections", img_bgr)
            cv2.waitKey(1)

        self.zed.ingest_custom_box_objects(custom_boxes)

        # 3. Get VIO Pose
        state = self.zed.get_position(self.cam_pose, sl.REFERENCE_FRAME.WORLD)
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            return

        # 4. Retrieve Raw 3D Objects
        self.zed.retrieve_objects(self.objects, self.obj_runtime_param)

        # 5. Debug - Table Log (If enabled)
        if self.debug_logs:
            self._log_debug_table(custom_boxes, self.objects.object_list)

        # 6. Prepare batch for tracker
        raw_measurements = []
        raw_covariances = []
        raw_classes = []

        # Rotation represents in matrix form, ideal for operations
        rotation_matrix = self.cam_pose.get_rotation_matrix().r
        
        for obj in self.objects.object_list:
            raw_pos = np.array(obj.position)
            if np.isnan(raw_pos).any() or np.isinf(raw_pos).any() or raw_pos[0] < 0:
                continue
            
            # Get label
            label_idx = obj.raw_label
            label_str = self.CLASS_NAMES[label_idx] if label_idx < len(self.CLASS_NAMES) else str(label_idx)

            # Transform position to World Frame
            world_pos = self._transform_pos_to_world(raw_pos, rotation_matrix)
            
            # Rotate covariance to World Frame: R @ Cov @ R.T
            try:
                cov_cam = np.array(obj.position_covariance).reshape(3, 3)
            except (AttributeError, ValueError):
                cov_cam = np.eye(3) * 0.1  # Fallback
                
            # Covariance Matrix requires outer product transformation
            # Refer to Variance in Expected Value definition for mathematical reason
            cov_world = rotation_matrix @ cov_cam @ rotation_matrix.T

            # Filter: Skip pipes detected more than 5m away (unreliable at distance)
            if label_str in ("red_pipe", "white_pipe"):
                # Distance from robot (camera position) to detection
                dist_from_robot = np.linalg.norm(raw_pos)  # raw_pos is in camera frame
                if dist_from_robot > 7.0:
                    continue  # Skip far-away pipe detections
            
            raw_measurements.append(world_pos)
            raw_covariances.append(cov_world)
            raw_classes.append(label_str)

        # 7. UPDATE TRACKER
        confirmed_tracks = self.tracker.update(raw_measurements, raw_covariances, raw_classes)

        # 8. Publish Confirmed Tracks
        msg = VisionObjectArray()
        for track in confirmed_tracks:
            vision_obj = VisionObject()
            vision_obj.label = track['class']
            vision_obj.id = track['id']  # Persistent ID from tracker
            
            # Use smoothed state from Kalman Filter
            vision_obj.x = float(track['kf'].x[0])
            vision_obj.y = float(track['kf'].x[1])
            vision_obj.z = float(track['kf'].x[2])
            vision_obj.theta_z = 0.0
            vision_obj.extra_field = 0.0
            
            # Confirmed track = high confidence
            vision_obj.confidence = 1.0

            msg.array.append(vision_obj)

        if msg.array:
            self.tracks_pub.publish(msg)
            
        self._publish_pose()

    def _log_debug_table(self, custom_boxes, zed_objects):
        """Print side-by-side comparison of YOLO input and ZED output."""
        yolo_labels = [self.CLASS_NAMES[b.label] for b in custom_boxes]
        
        zed_objs_labels = []
        for obj in zed_objects:
            lbl = self.CLASS_NAMES[obj.raw_label] if obj.raw_label < len(self.CLASS_NAMES) else str(obj.raw_label)
            zed_objs_labels.append(lbl)
            
        # Clear screen and move cursor to top
        print("\033[2J\033[H", end="")
        
        print("=" * 60)
        print(f"{'YOLO INPUT (Raw)':<30} | {'ZED OUTPUT (3D)':<28}")
        print("-" * 60)
        max_len = max(len(yolo_labels), len(zed_objs_labels))
        for i in range(max_len):
            y_lbl = yolo_labels[i] if i < len(yolo_labels) else ""
            z_lbl = zed_objs_labels[i] if i < len(zed_objs_labels) else ""
            print(f"{y_lbl:<30} | {z_lbl:<28}")
        print("=" * 60)
        print("\n")

    def _transform_pos_to_world(self, local_pos: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
        """Transform camera-frame position to world frame with sensor depth override."""
        translation = self.cam_pose.get_translation().get()
        translation[2] = -self.sensor_depth  # Override VIO Z with pressure sensor
        
        # TODO: Note sure if this how you transform the matrix accurately
        return translation + (rotation_matrix @ local_pos)

    def _publish_pose(self):
        """Publish VIO pose with sensor depth override."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        trans = self.cam_pose.get_translation().get()
        pose_msg.pose.position.x = float(trans[0])
        pose_msg.pose.position.y = float(trans[1])
        pose_msg.pose.position.z = -self.sensor_depth
        
        orient = self.cam_pose.get_orientation().get()
        pose_msg.pose.orientation.x = float(orient[0])
        pose_msg.pose.orientation.y = float(orient[1])
        pose_msg.pose.orientation.z = float(orient[2])
        pose_msg.pose.orientation.w = float(orient[3])
        
        self.pose_pub.publish(pose_msg)

    def _depth_callback(self, msg: Float64):
        """Update sensor depth (positive = down)."""
        self.sensor_depth = msg.data

    def destroy_node(self):
        if hasattr(self, 'zed'):
            self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Vision3DNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()