# Python dependencies
import py_trees
import threading

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
import geometry_msgs.msg
import auv_msgs.msg

# AUV dependencies
from controls import navigation_client

# Planner dependencies
from .missions.mission_sequence import DynamicMissionSequence

# I like my ANSI colours :DDD
green_text = "\033[32m"
default_text = "\033[0m"

def main():
    rclpy.init()
    node = rclpy.create_node("planner_root_tree")

    # Get tolerance, timeout and tick rate parameters from the configs
    node.declare_parameter("pre_qual.yaw_tolerance", 1.0)
    node.declare_parameter("pre_qual.positional_tolerance", 1.0)
    node.declare_parameter("pre_qual.timeout", 1.0)
    node.declare_parameter("pre_qual.hold_time", 1.0)
    node.declare_parameter("tick_rate", 1.0)

    pre_qual_yaw_tolerance = node.get_parameter("pre_qual.yaw_tolerance").get_parameter_value().double_value
    pre_qual_positional_tolerance = node.get_parameter("pre_qual.positional_tolerance").get_parameter_value().double_value
    pre_qual_timeout = node.get_parameter("pre_qual.timeout").get_parameter_value().double_value
    pre_qual_hold_time = node.get_parameter("pre_qual.hold_time").get_parameter_value().double_value
    tick_rate = node.get_parameter("tick_rate").get_parameter_value().double_value

    node.declare_parameter("pre_qual.orbit.yaw_tolerance_scale", 1.0)
    node.declare_parameter("pre_qual.orbit.positional_tolerance_scale", 1.0)
    node.declare_parameter("pre_qual.orbit.hold_time_initial", 1.0)
    node.declare_parameter("pre_qual.orbit.hold_time_segments", 1.0)

    orbit_pre_qual_yaw_tolerance_scale = node.get_parameter("pre_qual.orbit.yaw_tolerance_scale").get_parameter_value().double_value
    orbit_pre_qual_positional_tolerance_scale = node.get_parameter("pre_qual.orbit.positional_tolerance_scale").get_parameter_value().double_value
    orbit_pre_qual_hold_time_initial = node.get_parameter("pre_qual.orbit.hold_time_initial").get_parameter_value().double_value
    orbit_pre_qual_hold_time_segments = node.get_parameter("pre_qual.orbit.hold_time_segments").get_parameter_value().double_value

    node.declare_parameter("auto_record.enabled", True)
    node.declare_parameter("auto_record.profile", "all")
    node.declare_parameter("auto_record.bag_prefix", "mission_")
    node.declare_parameter("auto_record.service_path", "/rosbag_manager/control")

    auto_record_params = {
        'enabled': node.get_parameter("auto_record.enabled").get_parameter_value().bool_value,
        'profile': node.get_parameter("auto_record.profile").get_parameter_value().string_value,
        'bag_prefix': node.get_parameter("auto_record.bag_prefix").get_parameter_value().string_value,
        'service_path': node.get_parameter("auto_record.service_path").get_parameter_value().string_value
    }

    # Set the root of the tree
    root = py_trees.composites.Parallel("Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))

    # Create navigation client instance as a singleton
    nav_client = navigation_client.NavigationClient(name="planner_nav_client")
    
    # QoS for best effort sensors
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    node.declare_parameter("sim", False)
    node.declare_parameter("use_ground_truth", False)

    use_sim = node.get_parameter("sim").get_parameter_value().bool_value
    use_ground_truth = node.get_parameter("use_ground_truth").get_parameter_value().bool_value

    topic_pose = "state/pose"
    topic_twist = "auv_frame/dvl/velocity"

    if use_sim and use_ground_truth: 
        topic_pose = "/auv/ground_truth/pose"
        topic_twist = "/auv/ground_truth/twist"

    # ToBlackboard Subscribers
    pose_subscriber = py_trees_ros.subscribers.ToBlackboard(
        name="PoseSubscriber",
        topic_name=topic_pose,
        topic_type=geometry_msgs.msg.PoseStamped,
        blackboard_variables={"/sensors/pose": None},
        initialise_variables={"/sensors/pose": None},
        qos_profile=qos,
    )

    twist_subscriber = py_trees_ros.subscribers.ToBlackboard(
        name="TwistSubscriber",
        topic_name=topic_twist,
        topic_type=geometry_msgs.msg.TwistStamped,
        blackboard_variables={"/sensors/twist": None},
        initialise_variables={"/sensors/twist": None},
        qos_profile=qos,
    )

    object_map_subscriber = py_trees_ros.subscribers.ToBlackboard(
        name="ObjectMapSubscriber",
        topic_name="/vision/object_map",
        topic_type=auv_msgs.msg.VisionObjectArray,
        blackboard_variables={"/vision/object_map": None},
        initialise_variables={"/vision/object_map": None},
        qos_profile=qos,
    )

    # Mission Sequence
    missions = DynamicMissionSequence(
        position_tolerance=pre_qual_positional_tolerance,
        yaw_tolerance=pre_qual_yaw_tolerance,
        hold_time=pre_qual_hold_time,
        timeout=pre_qual_timeout,
        orbit_pre_qual_yaw_tolerance_scale=orbit_pre_qual_yaw_tolerance_scale,
        orbit_pre_qual_positional_tolerance_scale=orbit_pre_qual_positional_tolerance_scale,
        orbit_pre_qual_hold_time_initial=orbit_pre_qual_hold_time_initial,
        orbit_pre_qual_hold_time_segments=orbit_pre_qual_hold_time_segments,
        auto_record_params=auto_record_params
    )

    # Add children to root
    root.add_children([pose_subscriber, twist_subscriber, object_map_subscriber, missions])

    # Create the behaviour tree and setup
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    tree.setup(node=node, shared_nav_client=nav_client, timeout=15.0)

    # Setup ticking timer on the tree's node
    tree.tick_tock(period_ms=int(1000.0 / tick_rate))

    # Spin the action client in a parallel executor thread
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nav_client)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.get_logger().info(f"{green_text}Yaw Behaviour Tree Node Initialized{default_text}")

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down yaw BT node")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()