import rclpy
from rclpy.node import Node

from auv_msgs.msg import VisionObjectArray 
from foxglove_msgs.msg import SceneUpdate, SceneEntity, SpherePrimitive, TextPrimitive, CubePrimitive, SceneEntityDeletion
from geometry_msgs.msg import Point, Vector3, Pose

class SphereConverterNode(Node):
    # Mapping of AUV object labels to RGBA colors for Foxglove visualization
    CATEGORY_COLORS = {
        'gate':         {'r': 1.0, 'g': 0.5, 'b': 0.0, 'a': 0.8}, # Orange
        'lane_marker':  {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 0.8}, # Yellow
        'red_pipe':     {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.8}, # Red
        'white_pipe':   {'r': 1.0, 'g': 1.0, 'b': 1.0, 'a': 0.8}, # White
        'octagon':      {'r': 0.5, 'g': 0.0, 'b': 0.5, 'a': 0.8}, # Purplej
        'table':        {'r': 0.6, 'g': 0.3, 'b': 0.1, 'a': 0.8}, # Brown
        'bin':          {'r': 0.0, 'g': 1.0, 'b': 1.0, 'a': 0.8}, # Cyan
        'board':        {'r': 0.5, 'g': 0.5, 'b': 0.5, 'a': 0.8}, # Gray
        'shark':        {'r': 0.0, 'g': 0.5, 'b': 1.0, 'a': 0.8}, # Light Blue
        'sawfish':      {'r': 0.0, 'g': 0.8, 'b': 0.2, 'a': 0.8}, # Green
    }

    def __init__(self):
        super().__init__('vision_to_foxglove_node')
        
        # Declare parameter for input topic, default to /vision/objects_3d
        self.declare_parameter('input_topic', '/vision/object_map')
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        
        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.declare_parameter('pool_floor_z', -2.1)
        self.pool_floor_z = self.get_parameter('pool_floor_z').get_parameter_value().double_value
        
        self.subscription = self.create_subscription(
            VisionObjectArray,
            input_topic,
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            SceneUpdate,
            '/foxglove/scene_spheres',
            10)
        
        self.get_logger().info(f"Vision to Foxglove converter started.")
        self.get_logger().info(f"Subscribed to  : {input_topic}")
        self.get_logger().info(f"Publishing to  : /foxglove/scene_spheres")
        self.get_logger().info(f"Using frame_id : {self.frame_id}")

    def listener_callback(self, msg):
        scene_update = SceneUpdate()
        
        # 1. Send an explicit "Delete All Entities" command to clear previous frame's objects
        wipe_cmd = SceneEntityDeletion()
        wipe_cmd.timestamp = msg.header.stamp
        wipe_cmd.type = SceneEntityDeletion().ALL # 1 = ALL (Delete all existing entities on the same topic)
        scene_update.deletions.append(wipe_cmd)
        
        # 2. Add the new entities
        for i, obj in enumerate(msg.array):
            entity = SceneEntity()
            entity.id = f"{obj.label}_{i}"
            
            # Use the new array header properties for accurate synchronization
            entity.frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
            entity.timestamp = msg.header.stamp

            sphere = SpherePrimitive()
            
            # We can now simply copy the standard geometry_msgs/Pose completely!
            sphere.pose = obj.pose
            
            # Size mapping (Use the dynamic size vector if set, otherwise fallback to 0.25 default)
            if obj.size.x > 0.01 and obj.size.y > 0.01 and obj.size.z > 0.01:
                sphere.size = obj.size
            else:
                sphere.size = Vector3(x=0.25, y=0.25, z=0.25)
            
            # Color mapping based on label category
            color_map = self.CATEGORY_COLORS.get(obj.label, {'r': 1.0, 'g': 0.0, 'b': 1.0, 'a': 0.8}) # Default Magenta
            sphere.color.r = color_map['r']
            sphere.color.g = color_map['g']
            sphere.color.b = color_map['b']
            sphere.color.a = color_map['a'] * obj.confidence  # Dim out low confidence objects

            entity.spheres.append(sphere)
            
            # Add a floating text label slightly above the object
            text_label = TextPrimitive()
            
            # Create a completely independent Pose for the text to avoid mutating the sphere's pose
            text_pose = Pose()
            text_pose.position.x = obj.pose.position.x
            text_pose.position.y = obj.pose.position.y
            text_pose.position.z = obj.pose.position.z + (sphere.size.z / 2.0) + 0.2
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

        
        # Add a static pool floor representation
        floor_entity = SceneEntity()
        floor_entity.id = "pool_floor"
        floor_entity.frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
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
        # scene_update.entities.append(floor_entity)

        self.publisher.publish(scene_update)

def main(args=None):
    rclpy.init(args=args)
    node = SphereConverterNode()
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
