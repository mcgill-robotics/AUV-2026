#!/usr/bin/env python3
"""
3D Vision Node - YOLO + ZED SDK Integration for ROS2
Publishes detected objects with 3D positions (World Frame)
to /vision/objects_3d and VIO pose to /vision/vio_pose.
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

class Vision3DNode(Node):
    """ROS2 node for 3D object detection using YOLO + ZED SDK."""

    CLASS_NAMES = [
        "gate", "lane_marker", "red_pipe", "white_pipe", "octagon",
        "table", "bin", "board", "shark", "sawfish"
    ]

    def __init__(self):
        super().__init__('vision_3d_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('use_stream', True)
        self.declare_parameter('stream_ip', '127.0.0.1')
        self.declare_parameter('stream_port', 30000)

        # Get parameters
        model_path = self.get_parameter('model_path').value
        if not model_path:
            # Adjust path as necessary for your ROS2 workspace structure
            model_path = str(Path.home() / 'auv_ws/src/model_pipeline/runs/detect/yolov11s_sim_dataset7/weights/best.pt')
            
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.max_range = self.get_parameter('max_range').value
        self.use_stream = self.get_parameter('use_stream').value
        self.stream_ip = self.get_parameter('stream_ip').value
        self.stream_port = self.get_parameter('stream_port').value

        self.objects_pub = self.create_publisher(VisionObjectArray, '/vision/objects_3d', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/vio_pose', 10)

        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)

        self.zed = self._init_zed()

        # Runtime objects
        self.runtime_params = sl.RuntimeParameters()
        # [FIX] Explicitly ask for CAMERA frame to do manual transform later
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA 
        
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.obj_runtime_param.detection_confidence_threshold = 35
        
        self.image = sl.Mat()
        self.cam_pose = sl.Pose()

        # Simple ID persistence (ZED ID -> ROS ID)
        self.next_id = 1
        
        # Depth sensor fusion (overrides VIO Z with pressure sensor)
        self.sensor_depth = 0.0  # Positive = down
        self.depth_sub = self.create_subscription(
            Float64, '/sensors/depth/data', self._depth_callback, 10)
        
        self.timer = self.create_timer(1.0 / 30.0, self._process_frame)
        self.get_logger().info('Vision 3D Node initialized')

    def _init_zed(self) -> sl.Camera:
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.depth_maximum_distance = self.max_range
        
        if self.use_stream:
            self.get_logger().info(f"Connecting to {self.stream_ip}:{self.stream_port}...")
            init_params.set_from_stream(self.stream_ip, self.stream_port)

        zed = sl.Camera()
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Error opening ZED: {err}')
            raise RuntimeError(f'Failed to open ZED camera: {err}')

        pos_param = sl.PositionalTrackingParameters()
        pos_param.enable_imu_fusion = True
        pos_param.enable_area_memory = True
        pos_param.set_floor_as_origin = False
        zed.enable_positional_tracking(pos_param)

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True
        obj_param.max_range = self.max_range # [FIX] Also set max range for object engine
        zed.enable_object_detection(obj_param)

        return zed

    def _process_frame(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        image_bgr = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)

        results = self.model.predict(image_bgr, verbose=False, conf=self.confidence_threshold)

        custom_boxes = []
        for r in results:
            for box in r.boxes:
                tmp = sl.CustomBoxObjectData()
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                tmp.bounding_box_2d = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
                tmp.label = int(box.cls)
                tmp.probability = float(box.conf)
                
                # [FIX] Do NOT set unique_object_id. Let ZED internal tracker handle ID association.
                # tmp.unique_object_id = sl.generate_unique_id() <-- DELETE THIS
                
                # [FIX] Enable static mode for underwater objects to reduce jitter
                tmp.is_static = True 
                
                tmp.is_grounded = False 
                custom_boxes.append(tmp)

        self.zed.ingest_custom_box_objects(custom_boxes)

        # 1. Get Robot Pose (Currently VIO, later DVL/EKF)
        # Note: We still need WORLD frame here to know where the robot IS
        state = self.zed.get_position(self.cam_pose, sl.REFERENCE_FRAME.WORLD)
        
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            return  # VIO Lost - Do not publish garbage coordinates

        # 2. Retrieve Objects (LOCAL Coords)
        objects = sl.Objects()
        self.zed.retrieve_objects(objects, self.obj_runtime_param)

        msg = VisionObjectArray()
        
        for obj in objects.object_list:
            if obj.tracking_state != sl.OBJECT_TRACKING_STATE.OK:
                continue

            # This is [x,y,z] relative to camera lens
            local_pos = np.array(obj.position)
            
            if np.isnan(local_pos).any() or np.isinf(local_pos).any():
                continue

            # Sanity checks in Local Frame
            if local_pos[0] < 0: # Behind camera
                continue

            # 3. Manual Transform to Global
            world_pos = self.transform_local_to_world(local_pos, self.cam_pose)

            # Use ZED's persistent ID directly. It handles temporal tracking.
            # No need for complex internal map logic in the Vision Node itself.
            # That logic belongs in a "Mapping Node".
            
            vision_obj = VisionObject()
            vision_obj.label = self.CLASS_NAMES[obj.raw_label] if obj.raw_label < len(self.CLASS_NAMES) else f"class_{obj.raw_label}"
            vision_obj.id = int(obj.id) # Use ZED ID
            vision_obj.x = float(world_pos[0])
            vision_obj.y = float(world_pos[1])
            vision_obj.z = float(world_pos[2])
            vision_obj.confidence = float(obj.confidence)
            
            msg.array.append(vision_obj)

        if len(msg.array) > 0:
            self.objects_pub.publish(msg)
        
        # Publish VIO pose for Unity visualization
        # Override Z with sensor depth (more accurate than VIO)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        # Position (use sensor depth for Z, negated since down=positive in sensor but Z-up in world)
        translation = self.cam_pose.get_translation().get()
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = -self.sensor_depth  # Sensor: down=+, World: up=+
        
        # Orientation (quaternion)
        orientation = self.cam_pose.get_orientation().get()
        pose_msg.pose.orientation.x = float(orientation[0])
        pose_msg.pose.orientation.y = float(orientation[1])
        pose_msg.pose.orientation.z = float(orientation[2])
        pose_msg.pose.orientation.w = float(orientation[3])
        
        self.pose_pub.publish(pose_msg)
    
    def _depth_callback(self, msg: Float64):
        """Update sensor depth (positive = down)."""
        self.sensor_depth = msg.data

    def transform_local_to_world(self, local_pos_array, robot_pose):
        """
        Transform local (camera-relative) position to world frame.
        Uses sensor depth instead of VIO Z for accuracy.
        """
        rotation_matrix = robot_pose.get_rotation_matrix().r

        # Get translation, but override Z with sensor depth
        robot_position = robot_pose.get_translation().get()
        robot_position[2] = -self.sensor_depth  # Sensor: down=+, World: up=+

        # Rotate the local vector
        rotated_object = np.dot(rotation_matrix, local_pos_array)

        # Add robot's position
        global_pos = robot_position + rotated_object

        return global_pos

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
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()