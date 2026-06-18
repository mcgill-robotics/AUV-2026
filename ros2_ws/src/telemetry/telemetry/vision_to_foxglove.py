import math

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from auv_msgs.msg import FoxgloveObject, FoxgloveObjectArray, VisionObjectArray
from foxglove_msgs.msg import (
    CubePrimitive,
    LinePrimitive,
    SceneEntity,
    SceneEntityDeletion,
    SceneUpdate,
    TextPrimitive,
    ModelPrimitive,
)
from geometry_msgs.msg import PointStamped, Pose, Vector3, Point


class SceneConverterNode(Node):
    # Mapping of AUV object labels to RGBA colors for Foxglove visualization
    CATEGORY_COLORS = {
        'gate':            {'r': 1.0,  'g': 0.5,  'b': 0.0,  'a': 0.8}, # Orange
        'lane_marker':     {'r': 1.0,  'g': 1.0,  'b': 0.0,  'a': 0.8}, # Yellow
        'red_pipe':        {'r': 1.0,  'g': 0.0,  'b': 0.0,  'a': 0.8}, # Red
        'white_pipe':      {'r': 1.0,  'g': 1.0,  'b': 1.0,  'a': 0.8}, # White
        'octagon':         {'r': 0.5,  'g': 0.0,  'b': 0.5,  'a': 0.8}, # Purple
        'table':           {'r': 0.6,  'g': 0.3,  'b': 0.1,  'a': 0.8}, # Brown
        'bin':             {'r': 0.0,  'g': 1.0,  'b': 1.0,  'a': 0.8}, # Cyan
        'board':           {'r': 0.5,  'g': 0.5,  'b': 0.5,  'a': 0.8}, # Gray
        'blood':           {'r': 0.50, 'g': 0.00, 'b': 0.00, 'a': 0.8}, # Deep Blood Maroon
        'fire':            {'r': 1.00, 'g': 0.55, 'b': 0.00, 'a': 0.8}, # Flame Orange
        'ambulance':       {'r': 0.95, 'g': 0.10, 'b': 0.30, 'a': 0.8}, # Cherry Red
        'firetruck':       {'r': 1.00, 'g': 0.00, 'b': 0.00, 'a': 0.8}, # Red
        'redcross_helmet': {'r': 1.0,  'g': 1.0,  'b': 1.0,  'a': 0.8}, # White
        'warning':         {'r': 1.0,  'g': 1.0,  'b': 1.0,  'a': 0.8}, # White
    }

    def __init__(self):
        super().__init__('vision_to_foxglove_node')
        
        # Declare parameter for input topic, default to /vision/object_map
        self.declare_parameter('input_topic', '/vision/object_map')
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        
        self.declare_parameter('frame_id', 'pool_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.declare_parameter('auv_frame_id', 'auv_link')
        self.auv_frame_id = self.get_parameter('auv_frame_id').get_parameter_value().string_value
        
        self.declare_parameter('pool_floor_z', -2.1)
        self.pool_floor_z = self.get_parameter('pool_floor_z').get_parameter_value().double_value

        self.declare_parameter('auv_model_url', 'package://telemetry/meshes/auv.glb')
        self.auv_model_url = self.get_parameter('auv_model_url').get_parameter_value().string_value
        
        self.declare_parameter('auv_model_roll', 0.0)
        self.auv_model_roll = self.get_parameter('auv_model_roll').get_parameter_value().double_value

        self.declare_parameter('auv_model_pitch', 0.0)
        self.auv_model_pitch = self.get_parameter('auv_model_pitch').get_parameter_value().double_value

        self.declare_parameter('auv_model_yaw', 0.0)
        self.auv_model_yaw = self.get_parameter('auv_model_yaw').get_parameter_value().double_value

        self.declare_parameter('auv_model_x', 0.0)
        self.auv_model_x = self.get_parameter('auv_model_x').get_parameter_value().double_value

        self.declare_parameter('auv_model_y', 0.0)
        self.auv_model_y = self.get_parameter('auv_model_y').get_parameter_value().double_value

        self.declare_parameter('auv_model_z', 0.0)
        self.auv_model_z = self.get_parameter('auv_model_z').get_parameter_value().double_value
        
        self.declare_parameter('publish_auv_model', True)
        self.publish_auv_model = self.get_parameter('publish_auv_model').get_parameter_value().bool_value
        
        # Load lane boundaries from vision_pipeline.yaml
        self.draw_lane_boundary = False
        try:
            vision_config_path = os.path.join(get_package_share_directory('vision'), 'config', 'vision_pipeline.yaml')
            with open(vision_config_path, 'r') as f:
                vision_config = yaml.safe_load(f)
            lane_config = vision_config.get('object_map', {}).get('ros__parameters', {}).get('lane_boundary', {})
            
            self.draw_lane_boundary = lane_config.get('enable', False)
            if self.draw_lane_boundary:
                self.lane_x_min = float(lane_config.get('x_min', -3.0))
                self.lane_x_max = float(lane_config.get('x_max', 25.0))
                self.lane_y_min = float(lane_config.get('y_min', -7.5))
                self.lane_y_max = float(lane_config.get('y_max', 7.5))
                self.get_logger().info(f"Lane boundaries enabled: X[{self.lane_x_min}, {self.lane_x_max}], Y[{self.lane_y_min}, {self.lane_y_max}]")
        except Exception as e:
            self.get_logger().warning(f"Could not load lane boundaries from vision_pipeline.yaml: {e}")

        # Subscriptions
        self.subscription = self.create_subscription(
            VisionObjectArray,
            input_topic,
            self.listener_callback,
            10)
        
        # Publishers
        self.scene_publisher = self.create_publisher(SceneUpdate, '/foxglove/scene_objects', 10)
        self.published_entity_ids = set()

        self.table_publisher = self.create_publisher(
            FoxgloveObjectArray,
            '/vision/foxglove_table',
            10)

        # TF2 listener for pool_link -> auv_link transform (published by state_aggregator)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f"Vision to Foxglove converter started.")
        self.get_logger().info(f"Subscribed to  : {input_topic}")
        self.get_logger().info(f"Publishing to  : /foxglove/scene_objects, /vision/foxglove_table")
        self.get_logger().info(f"Using frame_id : {self.frame_id}")

    def _transform_point_to_auv(self, position, transform):
        """Transform a point from pool_link frame to auv_link frame using tf2."""
        point_stamped = PointStamped()
        point_stamped.point = position
        transformed = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        return transformed.point

    def listener_callback(self, msg):
        scene_update = SceneUpdate()
        
        # 1. Send an explicit "Delete All Entities" command to clear previous frame's objects
        wipe_cmd = SceneEntityDeletion()
        wipe_cmd.timestamp = msg.header.stamp
        wipe_cmd.type = SceneEntityDeletion().ALL # 1 = ALL (Delete all existing entities on the same topic)
        scene_update.deletions.append(wipe_cmd)

        table_msg = FoxgloveObjectArray()
        table_msg.header = msg.header
        
        frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
        
        # Lookup transform from pool_link -> auv_link (published by state_aggregator in sensors)
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.auv_frame_id,
                frame_id,
                rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            pass

        # Track IDs for this frame
        current_frame_ids = set()

        # 2. Add the new entities
        for obj in msg.array:
            # --- Table Logic ---
            fobj = FoxgloveObject()
            fobj.label = obj.label
            fobj.confidence = float(round(obj.confidence, 2))
            fobj.x_abs = float(round(obj.pose.position.x, 2))
            fobj.y_abs = float(round(obj.pose.position.y, 2))
            fobj.z_abs = float(round(obj.pose.position.z, 2))
            
            if transform is not None:
                p_rel = self._transform_point_to_auv(obj.pose.position, transform)
                fobj.x_rel = float(round(p_rel.x, 2))
                fobj.y_rel = float(round(p_rel.y, 2))
                fobj.z_rel = float(round(p_rel.z, 2))
            else:
                fobj.x_rel = float(round(fobj.x_abs, 2))
                fobj.y_rel = float(round(fobj.y_abs, 2))
                fobj.z_rel = float(round(fobj.z_abs, 2))
                
            fobj.distance = float(round(math.sqrt(fobj.x_rel**2 + fobj.y_rel**2 + fobj.z_rel**2), 2))
            table_msg.objects.append(fobj)
            # -------------------

            entity = SceneEntity()
            entity.id = f"{obj.label}_{obj.id}"
            current_frame_ids.add(entity.id)
            
            # Use the new array header properties for accurate synchronization
            entity.frame_id = frame_id
            entity.timestamp = msg.header.stamp

            # Color mapping based on label category
            color_map = self.CATEGORY_COLORS.get(obj.label, {'r': 1.0, 'g': 0.0, 'b': 1.0, 'a': 0.8}) # Default Magenta

            cube = CubePrimitive()
            cube.pose = obj.pose
            
            # Size mapping (Use the dynamic size vector if set, otherwise fallback to 0.25 default)
            if obj.size.x > 0.01 and obj.size.y > 0.01 and obj.size.z > 0.01:
                cube.size = obj.size
            else:
                cube.size = Vector3(x=0.25, y=0.25, z=0.25)
            
            cube.color.r = color_map['r']
            cube.color.g = color_map['g']
            cube.color.b = color_map['b']
            cube.color.a = color_map['a'] 

            entity.cubes.append(cube)
            
            # Add a floating text label slightly above the object
            text_label = TextPrimitive()
            
            # Create a completely independent Pose for the text to avoid mutating the object's pose
            text_pose = Pose()
            text_pose.position.x = obj.pose.position.x
            text_pose.position.y = obj.pose.position.y
            
            z_offset = (obj.size.z / 2.0) if (obj.size.z > 0.01) else 0.25
            text_pose.position.z = obj.pose.position.z + z_offset + 0.25
            text_pose.orientation = obj.pose.orientation
            
            text_label.pose = text_pose
            
            text_label.billboard = True # Always face the camera
            text_label.font_size = 0.15
            text_label.scale_invariant = False
            
            text_label.color.r = 1.0
            text_label.color.g = 1.0
            text_label.color.b = 1.0
            text_label.color.a = 0.6
            
            text_label.text = f"{obj.label} ({int(obj.confidence*100)}%)"

            entity.texts.append(text_label)
            scene_update.entities.append(entity)

        # 1. Send explicit deletions ONLY for entities that disappeared
        # deleted_ids = self.published_entity_ids - current_frame_ids
        # for del_id in deleted_ids:
        #     wipe_cmd = SceneEntityDeletion()
        #     wipe_cmd.timestamp = msg.header.stamp
        #     wipe_cmd.type = SceneEntityDeletion.MATCHING_ID
        #     wipe_cmd.id = del_id
        #     scene_update.deletions.append(wipe_cmd)
            
        self.published_entity_ids = current_frame_ids
        
        # Add a static pool floor representation
        floor_entity = SceneEntity()
        floor_entity.id = "pool_floor"
        floor_entity.frame_id = frame_id
        floor_entity.timestamp = msg.header.stamp
        
        floor_cube = CubePrimitive()
        floor_cube.pose.position.x = 0.0
        floor_cube.pose.position.y = 0.0
        floor_cube.pose.position.z = self.pool_floor_z
        floor_cube.pose.orientation.w = 1.0
        
        # Make it wide and long (20m x 20m), but very thin
        floor_cube.size = Vector3(x=50.0, y=50.0, z=0.01)
        
        # A faint blue/grey for the pool floor
        floor_cube.color.r = 0.2
        floor_cube.color.g = 0.4
        floor_cube.color.b = 0.6
        floor_cube.color.a = 0.3
        
        floor_entity.cubes.append(floor_cube)
        scene_update.entities.append(floor_entity)

        # Draw Lane Boundaries if enabled
        if getattr(self, 'draw_lane_boundary', False):
            lane_entity = SceneEntity()
            lane_entity.id = "lane_boundary"
            lane_entity.frame_id = frame_id
            lane_entity.timestamp = msg.header.stamp
            
            lane_line = LinePrimitive()
            lane_line.type = LinePrimitive.LINE_STRIP
            lane_line.pose.orientation.w = 1.0
            lane_line.thickness = 0.1 # 10cm thick line
            lane_line.scale_invariant = False
            
            # Bright Green
            lane_line.color.r = 0.0
            lane_line.color.g = 1.0
            lane_line.color.b = 0.0
            lane_line.color.a = 0.8
            
            z_plane = self.pool_floor_z + 0.05 # Slightly above the floor to avoid z-fighting
            
            # Create rectangle
            pts = [
                Point(x=self.lane_x_min, y=self.lane_y_min, z=z_plane),
                Point(x=self.lane_x_max, y=self.lane_y_min, z=z_plane),
                Point(x=self.lane_x_max, y=self.lane_y_max, z=z_plane),
                Point(x=self.lane_x_min, y=self.lane_y_max, z=z_plane),
                Point(x=self.lane_x_min, y=self.lane_y_min, z=z_plane) # Close the loop
            ]
            lane_line.points = pts
            
            lane_entity.lines.append(lane_line)
            scene_update.entities.append(lane_entity)

        # Add the AUV 3D Model
        if self.publish_auv_model:
            auv_entity = SceneEntity()
            auv_entity.id = "auv_3d_model"
            auv_entity.timestamp = msg.header.stamp
            
            auv_model = ModelPrimitive()
            
            # Base pose in AUV frame
            auv_pose = Pose()
            auv_pose.position.x = self.auv_model_x
            auv_pose.position.y = self.auv_model_y
            auv_pose.position.z = self.auv_model_z

            # Compute quaternion from Euler angles using tf_transformations
            q = quaternion_from_euler(self.auv_model_roll, self.auv_model_pitch, self.auv_model_yaw)
            auv_pose.orientation.x = q[0]
            auv_pose.orientation.y = q[1]
            auv_pose.orientation.z = q[2]
            auv_pose.orientation.w = q[3]

            if transform is not None:
                try:
                    # Lookup transform from auv_link -> pool_link
                    auv_to_pool_tf = self.tf_buffer.lookup_transform(
                        frame_id,          # Target (pool_link)
                        self.auv_frame_id, # Source (auv_link)
                        rclpy.time.Time()
                    )
                    
                    # Transform from auv_link to pool_link
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(auv_pose, auv_to_pool_tf)
                    auv_model.pose = transformed_pose
                    auv_entity.frame_id = frame_id
                except tf2_ros.TransformException:
                    auv_model.pose = auv_pose
                    auv_entity.frame_id = self.auv_frame_id
            else:
                auv_model.pose = auv_pose
                auv_entity.frame_id = self.auv_frame_id
            
            auv_model.scale = Vector3(x=1.0, y=1.0, z=1.0)
            auv_model.url = self.auv_model_url
            auv_model.override_color = False
            
            auv_entity.models.append(auv_model)
            scene_update.entities.append(auv_entity)

        self.table_publisher.publish(table_msg)
        self.scene_publisher.publish(scene_update)

def main(args=None):
    rclpy.init(args=args)
    node = SceneConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
