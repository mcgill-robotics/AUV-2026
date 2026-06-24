# Python dependencies
import math
import py_trees
import threading

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
import geometry_msgs.msg
import auv_msgs.msg
from rclpy.parameter import Parameter

# AUV dependencies
from controls import navigation_client
import std_msgs.msg

# Planner dependencies
from .missions.mission_sequence import DynamicMissionSequence

# I like my ANSI colours :DDD
green_text = "\033[32m"
default_text = "\033[0m"

def main():
    rclpy.init()
    node = rclpy.create_node("planner_root_tree")

    # Get tolerance, timeout and tick rate parameters from the configs
    node.declare_parameter("pre_qual.angular_tolerance", 1.0)
    node.declare_parameter("pre_qual.positional_tolerance", 1.0)
    node.declare_parameter("pre_qual.timeout", 1.0)
    node.declare_parameter("pre_qual.hold_time", 1.0)
    node.declare_parameter("tick_rate", 1.0)

    pre_qual_angular_tolerance = node.get_parameter("pre_qual.angular_tolerance").get_parameter_value().double_value
    pre_qual_positional_tolerance = node.get_parameter("pre_qual.positional_tolerance").get_parameter_value().double_value
    pre_qual_timeout = node.get_parameter("pre_qual.timeout").get_parameter_value().double_value
    pre_qual_hold_time = node.get_parameter("pre_qual.hold_time").get_parameter_value().double_value
    tick_rate = node.get_parameter("tick_rate").get_parameter_value().double_value

    node.declare_parameter("pre_qual.orbit.angular_tolerance_scale", 1.0)
    node.declare_parameter("pre_qual.orbit.positional_tolerance_scale", 1.0)
    node.declare_parameter("pre_qual.orbit.hold_time_initial", 1.0)
    node.declare_parameter("pre_qual.orbit.hold_time_segments", 1.0)

    orbit_pre_qual_angular_tolerance_scale = node.get_parameter("pre_qual.orbit.angular_tolerance_scale").get_parameter_value().double_value
    orbit_pre_qual_positional_tolerance_scale = node.get_parameter("pre_qual.orbit.positional_tolerance_scale").get_parameter_value().double_value
    orbit_pre_qual_hold_time_initial = node.get_parameter("pre_qual.orbit.hold_time_initial").get_parameter_value().double_value
    orbit_pre_qual_hold_time_segments = node.get_parameter("pre_qual.orbit.hold_time_segments").get_parameter_value().double_value

    # Slalom task parameters
    node.declare_parameter("slalom.num_layers", 3)
    node.declare_parameter("slalom.gate_side", "right")
    node.declare_parameter("slalom.scan_angle_deg", 60.0)
    node.declare_parameter("slalom.scan_pause_time", 1.0)
    node.declare_parameter("slalom.collinearity_threshold", 0.5)
    node.declare_parameter("slalom.min_forward_dist", 0.5)
    node.declare_parameter("slalom.layer_distance", 2.0)
    node.declare_parameter("slalom.initial_depth", -1.0)
    node.declare_parameter("slalom.position_tolerance", 0.3)
    node.declare_parameter("slalom.angular_tolerance_deg", 17.0)
    node.declare_parameter("slalom.hold_time", 0.5)
    node.declare_parameter("slalom.timeout", 45.0)
    node.declare_parameter("slalom.scan_angular_tolerance_deg", 30.0)
    node.declare_parameter("slalom.scan_hold_time", 0.1)
    node.declare_parameter("slalom.scan_timeout", 30.0)
    
    # Gate task parameters
    node.declare_parameter("gate.position_tolerance", 0.3)
    node.declare_parameter("gate.hold_time", 1.0)
    node.declare_parameter("gate.timeout", 30.0)
    node.declare_parameter("gate.desired_role", "survey_repair")
    node.declare_parameter("gate.search_attempts", 2)
    node.declare_parameter("gate.scan_angle_deg", 35.0)
    node.declare_parameter("gate.scan_pause_time", 1.0)
    node.declare_parameter("gate.approach_distance", 1.0)
    node.declare_parameter("gate.pass_distance", 1.0)
    
    # Bins task parameters
    node.declare_parameter("bins.downcam_fov_horizontal", 59.7)
    node.declare_parameter("bins.downcam_fov_vertical", 47.6)
    node.declare_parameter("bins.downcam_image_width", 640)
    node.declare_parameter("bins.downcam_image_height", 480)
    node.declare_parameter("bins.search_sweep_steps", 8)
    node.declare_parameter("bins.search_sweep_step_timeout", 0.5)
    node.declare_parameter("bins.bin_moving_average_weight", 0.5)
    node.declare_parameter("bins.bin_structure_distance", 2.0)
    node.declare_parameter("bins.go_above_bin_structure_height", 0.5)
    node.declare_parameter("bins.go_above_bin_height", 0.7)
    node.declare_parameter("bins.switch_sides_height", 1.2)
    node.declare_parameter("bins.wrong_task_type_threshold", 5)
    node.declare_parameter("bins.task_completion_threshold", 5)
    node.declare_parameter("bins.bin_lined_up_threshold", 10)
    node.declare_parameter("bins.num_required_markers", 2)
    node.declare_parameter("bins.num_bins", 4)
    node.declare_parameter("bins.bins_to_bin_structure", 0.3)
    node.declare_parameter("bins.force_fallback_search", False)
    node.declare_parameter("bins.force_fallback_alignment", False)
    
    # Torpedo task parameters
    node.declare_parameter("torpedo.initial_distance_from_board", Parameter.Type.DOUBLE)
    node.declare_parameter("torpedo.z_reference", Parameter.Type.DOUBLE)
    node.declare_parameter("torpedo.scan_pause_time", Parameter.Type.DOUBLE)
    node.declare_parameter("torpedo.position_tolerance", Parameter.Type.DOUBLE)
    node.declare_parameter("torpedo.yaw_tolerance_deg", Parameter.Type.DOUBLE)
    node.declare_parameter("torpedo.hold_time", Parameter.Type.DOUBLE)
    node.declare_parameter("torpedo.timeout", Parameter.Type.DOUBLE)
    node.declare_parameter("torpedo.refinement.rejection_threshold_deg", Parameter.Type.DOUBLE)
    node.declare_parameter("torpedo.refinement.attempts", Parameter.Type.INTEGER)
    node.declare_parameter("torpedo.refinement.alignments_per_attempt", Parameter.Type.INTEGER)
    node.declare_parameter("torpedo.refinement.samples_per_alignment", Parameter.Type.INTEGER)
    node.declare_parameter("torpedo.refinement.sample_every_n_ticks", Parameter.Type.INTEGER)
    node.declare_parameter("torpedo.auv_to_torpedos.left", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.auv_to_torpedos.right", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.torpedo_trajectory_coefficients.x", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.torpedo_trajectory_coefficients.y", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.torpedo_trajectory_coefficients.z", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.icon_to_nearest_hole.board_1.blood", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.icon_to_nearest_hole.board_1.fire", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.icon_to_nearest_hole.board_1.ambulance", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.icon_to_nearest_hole.board_1.firetruck", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.icon_to_nearest_hole.board_2.blood", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.icon_to_nearest_hole.board_2.fire", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.icon_to_nearest_hole.board_2.ambulance", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("torpedo.icon_to_nearest_hole.board_2.firetruck", Parameter.Type.DOUBLE_ARRAY)
    
    slalom_params = {
        "num_layers": node.get_parameter("slalom.num_layers").get_parameter_value().integer_value,
        "gate_side": node.get_parameter("slalom.gate_side").get_parameter_value().string_value,
        "scan_angle_deg": node.get_parameter("slalom.scan_angle_deg").get_parameter_value().double_value,
        "scan_pause_time": node.get_parameter("slalom.scan_pause_time").get_parameter_value().double_value,
        "collinearity_threshold": node.get_parameter("slalom.collinearity_threshold").get_parameter_value().double_value,
        "min_forward_dist": node.get_parameter("slalom.min_forward_dist").get_parameter_value().double_value,
        "layer_distance": node.get_parameter("slalom.layer_distance").get_parameter_value().double_value,
        "initial_depth": node.get_parameter("slalom.initial_depth").get_parameter_value().double_value,
        "position_tolerance": node.get_parameter("slalom.position_tolerance").get_parameter_value().double_value,
        "angular_tolerance_rad": math.radians(node.get_parameter("slalom.angular_tolerance_deg").get_parameter_value().double_value),  # deg -> rad
        "hold_time": node.get_parameter("slalom.hold_time").get_parameter_value().double_value,
        "timeout": node.get_parameter("slalom.timeout").get_parameter_value().double_value,
        "scan_angular_tolerance_rad": math.radians(node.get_parameter("slalom.scan_angular_tolerance_deg").get_parameter_value().double_value),  # deg -> rad
        "scan_hold_time": node.get_parameter("slalom.scan_hold_time").get_parameter_value().double_value,
        "scan_timeout": node.get_parameter("slalom.scan_timeout").get_parameter_value().double_value,
    }


    gate_params = {
        "position_tolerance": node.get_parameter("gate.position_tolerance").get_parameter_value().double_value,
        "hold_time": node.get_parameter("gate.hold_time").get_parameter_value().double_value,
        "timeout": node.get_parameter("gate.timeout").get_parameter_value().double_value,
        "desired_role": node.get_parameter("gate.desired_role").get_parameter_value().string_value,
        "search_attempts": node.get_parameter("gate.search_attempts").get_parameter_value().integer_value,
        "scan_angle_deg": node.get_parameter("gate.scan_angle_deg").get_parameter_value().double_value,
        "scan_pause_time": node.get_parameter("gate.scan_pause_time").get_parameter_value().double_value,
        "approach_distance": node.get_parameter("gate.approach_distance").get_parameter_value().double_value,
        "pass_distance": node.get_parameter("gate.pass_distance").get_parameter_value().double_value,
    }


    bins_params = {
        "downcam_fov_horizontal": node.get_parameter("bins.downcam_fov_horizontal").get_parameter_value().double_value,
        "downcam_fov_vertical": node.get_parameter("bins.downcam_fov_vertical").get_parameter_value().double_value,
        "downcam_image_width": node.get_parameter("bins.downcam_image_width").get_parameter_value().integer_value,
        "downcam_image_height": node.get_parameter("bins.downcam_image_height").get_parameter_value().integer_value,
        "search_sweep_steps": node.get_parameter("bins.search_sweep_steps").get_parameter_value().integer_value,
        "search_sweep_step_timeout": node.get_parameter("bins.search_sweep_step_timeout").get_parameter_value().double_value,
        "bin_moving_average_weight": node.get_parameter("bins.bin_moving_average_weight").get_parameter_value().double_value,
        "bin_structure_distance": node.get_parameter("bins.bin_structure_distance").get_parameter_value().double_value,
        "go_above_bin_structure_height": node.get_parameter("bins.go_above_bin_structure_height").get_parameter_value().double_value,
        "go_above_bin_height": node.get_parameter("bins.go_above_bin_height").get_parameter_value().double_value,
        "switch_sides_height": node.get_parameter("bins.switch_sides_height").get_parameter_value().double_value,
        "wrong_task_type_threshold": node.get_parameter("bins.wrong_task_type_threshold").get_parameter_value().integer_value,
        "task_completion_threshold": node.get_parameter("bins.task_completion_threshold").get_parameter_value().integer_value,
        "bin_lined_up_threshold": node.get_parameter("bins.bin_lined_up_threshold").get_parameter_value().integer_value,
        "num_required_markers": node.get_parameter("bins.num_required_markers").get_parameter_value().integer_value,
        "num_bins": node.get_parameter("bins.num_bins").get_parameter_value().integer_value,
        "bins_to_bin_structure": node.get_parameter("bins.bins_to_bin_structure").get_parameter_value().double_value,
        "force_fallback_search": node.get_parameter("bins.force_fallback_search").get_parameter_value().bool_value,
        "force_fallback_alignment": node.get_parameter("bins.force_fallback_alignment").get_parameter_value().bool_value,
    }
    
    torpedo_params = {
        "initial_distance_from_board": node.get_parameter("torpedo.initial_distance_from_board").get_parameter_value().double_value,
        "z_reference": node.get_parameter("torpedo.z_reference").get_parameter_value().double_value,
        "scan_pause_time": node.get_parameter("torpedo.scan_pause_time").get_parameter_value().double_value,
        "position_tolerance": node.get_parameter("torpedo.position_tolerance").get_parameter_value().double_value,
        "yaw_tolerance_rad": math.radians(node.get_parameter("torpedo.yaw_tolerance_deg").get_parameter_value().double_value),  # deg -> rad
        "hold_time": node.get_parameter("torpedo.hold_time").get_parameter_value().double_value,
        "timeout": node.get_parameter("torpedo.timeout").get_parameter_value().double_value,
        "refinement_rejection_threshold_rad": math.radians(node.get_parameter("torpedo.refinement.rejection_threshold_deg").get_parameter_value().double_value),  # deg -> rad
        "refinement_attempts" : node.get_parameter("torpedo.refinement.attempts").get_parameter_value().integer_value,
        "alignments_per_attempt": node.get_parameter("torpedo.refinement.alignments_per_attempt").get_parameter_value().integer_value,
        "samples_per_alignment": node.get_parameter("torpedo.refinement.samples_per_alignment").get_parameter_value().integer_value,
        "refinement_sample_every_n_ticks": node.get_parameter("torpedo.refinement.sample_every_n_ticks").get_parameter_value().integer_value,
        "auv_to_torpedos": {
            "left": node.get_parameter("torpedo.auv_to_torpedos.left").get_parameter_value().double_array_value,
            "right": node.get_parameter("torpedo.auv_to_torpedos.right").get_parameter_value().double_array_value
        },
        "torpedo_trajectory_coefficients": {
            "x": node.get_parameter("torpedo.torpedo_trajectory_coefficients.x").get_parameter_value().double_array_value,
            "y": node.get_parameter("torpedo.torpedo_trajectory_coefficients.y").get_parameter_value().double_array_value,
            "z": node.get_parameter("torpedo.torpedo_trajectory_coefficients.z").get_parameter_value().double_array_value
        },
        "icon_to_nearest_hole": {
            "board_1": {
                "blood": node.get_parameter("torpedo.icon_to_nearest_hole.board_1.blood").get_parameter_value().double_array_value,
                "fire": node.get_parameter("torpedo.icon_to_nearest_hole.board_1.fire").get_parameter_value().double_array_value,
                "ambulance": node.get_parameter("torpedo.icon_to_nearest_hole.board_1.ambulance").get_parameter_value().double_array_value,
                "firetruck": node.get_parameter("torpedo.icon_to_nearest_hole.board_1.firetruck").get_parameter_value().double_array_value
            },
            "board_2": {
                "blood": node.get_parameter("torpedo.icon_to_nearest_hole.board_2.blood").get_parameter_value().double_array_value,
                "fire": node.get_parameter("torpedo.icon_to_nearest_hole.board_2.fire").get_parameter_value().double_array_value,
                "ambulance": node.get_parameter("torpedo.icon_to_nearest_hole.board_2.ambulance").get_parameter_value().double_array_value,
                "firetruck": node.get_parameter("torpedo.icon_to_nearest_hole.board_2.firetruck").get_parameter_value().double_array_value
            }
        }
    }

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

    down_cam_subscriber = py_trees_ros.subscribers.ToBlackboard(
        name="DownCamSubscriber",
        topic_name="/vision/down_cam/detection_frame",
        topic_type=auv_msgs.msg.VisionDetectionFrame,
        blackboard_variables={"/vision/down_cam/detections": None},
        initialise_variables={"/vision/down_cam/detections": None},
        qos_profile=qos,
    )
    
    torpedo_launch_publisher = py_trees_ros.publishers.FromBlackboard(
        name="TorpedoLaunchPublisher",
        topic_name=node.get_parameter("torpedo.launch_topic").get_parameter_value().string_value,
        topic_type=std_msgs.msg.UInt8,
        blackboard_variable="/torpedo/launch_command",
        qos_profile=qos,
    )

    # Mission Sequence
    missions = DynamicMissionSequence(
        position_tolerance=pre_qual_positional_tolerance,
        angular_tolerance=pre_qual_angular_tolerance,
        hold_time=pre_qual_hold_time,
        timeout=pre_qual_timeout,
        orbit_pre_qual_angular_tolerance_scale=orbit_pre_qual_angular_tolerance_scale,
        orbit_pre_qual_positional_tolerance_scale=orbit_pre_qual_positional_tolerance_scale,
        orbit_pre_qual_hold_time_initial=orbit_pre_qual_hold_time_initial,
        orbit_pre_qual_hold_time_segments=orbit_pre_qual_hold_time_segments,
        slalom_params=slalom_params,
        torpedo_params=torpedo_params,
        gate_params=gate_params,
        bins_params=bins_params,
        auto_record_params=auto_record_params
    )

    # Add children to root
    root.add_children([pose_subscriber, twist_subscriber, object_map_subscriber, down_cam_subscriber, torpedo_launch_publisher, missions])

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