#include "vision/object_map.hpp"

using namespace std;

ObjectMapNode::ObjectMapNode() : Node("object_map_node")
{
    RCLCPP_INFO(this->get_logger(), "[INIT] ObjectMapNode constructor started");

    string front_cam_detection_frame_topic =
        this->declare_parameter<string>("front_cam_detection_frame_topic");
    string down_cam_detection_topic = 
        this->declare_parameter<string>("down_cam_detection_topic");
    string down_cam_info_topic = 
        this->declare_parameter<string>("down_cam_info_topic");
    string object_map_topic = this->declare_parameter<string>("object_map_topic");
    auv_frame_id = this->declare_parameter<string>("frame_id_auv");
    
    down_cam_fx = 0.0;
    down_cam_fy = 0.0;
    down_cam_cx = 0.0;
    down_cam_cy = 0.0;


    float new_object_min_distance_threshold =
        this->declare_parameter<float>("new_object_min_distance_threshold");
    bool large_structure_separation_enabled = this->declare_parameter<bool>("large_structure_separation.enable");
    float min_large_structure_separation =
        this->declare_parameter<float>("large_structure_separation.min_distance_m");
    bool large_structure_pipe_separation_enabled = this->declare_parameter<bool>("large_structure_pipe_separation.enable");
    float min_large_structure_pipe_separation =
        this->declare_parameter<float>("large_structure_pipe_separation.min_distance_m");
    double front_gating_threshold = this->declare_parameter<double>("front_tracker.gating_threshold");
    int front_min_hits = this->declare_parameter<int>("front_tracker.min_hits");
    int front_max_age = this->declare_parameter<int>("front_tracker.max_age");
    double front_max_position_jump = this->declare_parameter<double>("front_tracker.max_position_jump");
    int front_conf_to_tent_threshold = this->declare_parameter<int>("front_tracker.conf_to_tent_threshold");
    int front_tent_init_buffer = this->declare_parameter<int>("front_tracker.tent_init_buffer");

    double down_gating_threshold = this->declare_parameter<double>("down_tracker.gating_threshold");
    int down_min_hits = this->declare_parameter<int>("down_tracker.min_hits");
    int down_max_age = this->declare_parameter<int>("down_tracker.max_age");
    double down_max_position_jump = this->declare_parameter<double>("down_tracker.max_position_jump");
    int down_conf_to_tent_threshold = this->declare_parameter<int>("down_tracker.conf_to_tent_threshold");
    int down_tent_init_buffer = this->declare_parameter<int>("down_tracker.tent_init_buffer");

    bool enable_gate_midpoint_refinement =
        this->declare_parameter<bool>("gate_midpoint");
    bool enable_board_icon_refinement =
        this->declare_parameter<bool>("board_icon");
    float refinement_plausibility_radius =
        this->declare_parameter<float>("refinement_plausibility_radius");

    std::vector<std::string> large_structure_labels =
        this->declare_parameter<std::vector<std::string>>("large_structures");
    std::vector<std::string> pipe_labels =
        this->declare_parameter<std::vector<std::string>>("pipes");
    std::vector<std::string> semi_persistent_objects = 
        this->declare_parameter<std::vector<std::string>>("semi_persistent_objects");
    int semi_persistent_conf_to_tent_threshold =
            this->declare_parameter<int>("semi_persistent_conf_to_tent_threshold");
    std::vector<std::string> max_per_class_labels =
        this->declare_parameter<std::vector<std::string>>("max_per_class_labels");
    std::vector<int64_t> max_per_class_values =
        this->declare_parameter<std::vector<int64_t>>("max_per_class_values");

    std::unordered_map<std::string, int> max_per_class_map;
    for (size_t i = 0; i < max_per_class_labels.size() && i < max_per_class_values.size(); ++i) {
        max_per_class_map[max_per_class_labels[i]] = static_cast<int>(max_per_class_values[i]);
    }

    std::vector<std::string> object_size_labels = this->declare_parameter<std::vector<std::string>>("object_size_labels", std::vector<std::string>());
    std::vector<double> object_size_x = this->declare_parameter<std::vector<double>>("object_size_x", std::vector<double>());
    std::vector<double> object_size_y = this->declare_parameter<std::vector<double>>("object_size_y", std::vector<double>());
    std::vector<double> object_size_z = this->declare_parameter<std::vector<double>>("object_size_z", std::vector<double>());

    for (size_t i = 0; i < object_size_labels.size() && i < object_size_x.size() && i < object_size_y.size() && i < object_size_z.size(); ++i) {
        object_sizes_map[object_size_labels[i]] = Eigen::Vector3d(object_size_x[i], object_size_y[i], object_size_z[i]);
    }

    std::vector<std::string> down_cam_projection_labels = this->declare_parameter<std::vector<std::string>>("down_cam_projection_labels", std::vector<std::string>());
    std::vector<double> down_cam_projection_heights = this->declare_parameter<std::vector<double>>("down_cam_projection_heights", std::vector<double>());

    for (size_t i = 0; i < down_cam_projection_labels.size() && i < down_cam_projection_heights.size(); ++i) {
        down_cam_heights_map[down_cam_projection_labels[i]] = down_cam_projection_heights[i];
    }

    front_tracker = ObjectTracker(
        max_per_class_map,
        large_structure_labels,
        pipe_labels,
        new_object_min_distance_threshold,
        large_structure_separation_enabled,
        min_large_structure_separation,
        large_structure_pipe_separation_enabled,
        min_large_structure_pipe_separation,
        front_gating_threshold,
        front_min_hits,
        front_max_age,
        front_max_position_jump,
        front_conf_to_tent_threshold,
        front_tent_init_buffer,
        semi_persistent_objects,
        semi_persistent_conf_to_tent_threshold,
        enable_gate_midpoint_refinement,
        enable_board_icon_refinement,
        refinement_plausibility_radius
    );

    down_tracker = ObjectTracker(
        max_per_class_map,
        large_structure_labels,
        pipe_labels,
        new_object_min_distance_threshold,
        large_structure_separation_enabled,
        min_large_structure_separation,
        large_structure_pipe_separation_enabled,
        min_large_structure_pipe_separation,
        down_gating_threshold,
        down_min_hits,
        down_max_age,
        down_max_position_jump,
        down_conf_to_tent_threshold,
        down_tent_init_buffer,
        semi_persistent_objects,
        semi_persistent_conf_to_tent_threshold,
        enable_gate_midpoint_refinement,
        enable_board_icon_refinement,
        refinement_plausibility_radius
    );

    enable_z_axis_locking = this->declare_parameter<bool>("z_axis_locking.enable");
    pool_floor_z = this->declare_parameter<double>("z_axis_locking.pool_floor_z");
    pool_surface_z = this->declare_parameter<double>("z_axis_locking.pool_surface_z");
    double table_height = this->declare_parameter<double>("table_height", 0.67);
    water_refraction_scale = this->declare_parameter<double>("water_refraction_scale", 1.33);
    table_z = pool_floor_z + table_height;
    table_octagon_refinement_mode = this->declare_parameter<std::string>("table_octagon_mode");
    unique_objects = this->declare_parameter<std::vector<std::string>>("unique_objects");
    floor_objects = this->declare_parameter<std::vector<std::string>>("floor_objects");
    surface_objects = this->declare_parameter<std::vector<std::string>>("surface_objects");
    table_top_objects = this->declare_parameter<std::vector<std::string>>("table_top_objects");
    enable_pipe_distance_truncation = this->declare_parameter<bool>("pipe_distance_truncation.enable");
    max_pipe_distance = this->declare_parameter<double>("pipe_distance_truncation.max_distance_m");
    enable_lane_boundary = this->declare_parameter<bool>("lane_boundary.enable", false);
    lane_x_min = this->declare_parameter<double>("lane_boundary.x_min", -100.0);
    lane_x_max = this->declare_parameter<double>("lane_boundary.x_max", 100.0);
    lane_y_min = this->declare_parameter<double>("lane_boundary.y_min", -100.0);
    lane_y_max = this->declare_parameter<double>("lane_boundary.y_max", 100.0);

    if (enable_lane_boundary) {
        RCLCPP_INFO(
            this->get_logger(),
            "Lane boundary enabled: x=[%.1f, %.1f] y=[%.1f, %.1f]",
            lane_x_min, lane_x_max, lane_y_min, lane_y_max);
    }

    object_map_publisher =
        this->create_publisher<auv_msgs::msg::VisionObjectArray>(object_map_topic, 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    detection_subscriber =
        this->create_subscription<auv_msgs::msg::VisionDetectionFrame>(
            front_cam_detection_frame_topic,
            10,
            std::bind(&ObjectMapNode::detection_callback, this, std::placeholders::_1));

    down_cam_subscriber = 
        this->create_subscription<vision_msgs::msg::Detection2DArray>(
            down_cam_detection_topic,
            10,
            std::bind(&ObjectMapNode::down_cam_callback, this, std::placeholders::_1));

    down_cam_info_subscriber = 
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            down_cam_info_topic,
            10,
            std::bind(&ObjectMapNode::down_cam_info_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Using synchronized front-camera detection frames for object mapping.");
}

Eigen::Vector3d ObjectMapNode::eigen_from_point(const geometry_msgs::msg::Point& point)
{
    return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector2d ObjectMapNode::xy(const Eigen::Vector3d& position)
{
    return position.head<2>();
}

Eigen::Matrix3d ObjectMapNode::covariance_from_ros_pose(const std::array<double, 36>& covariance)
{
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.1;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            cov(r, c) = covariance[r * 6 + c];
        }
    }
    return cov;
}

bool ObjectMapNode::is_unique_object(const std::string& label) const
{
    return std::find(unique_objects.begin(), unique_objects.end(), label) != unique_objects.end();
}

bool ObjectMapNode::is_floor_bound(const std::string& label) const
{
    return std::find(floor_objects.begin(), floor_objects.end(), label) != floor_objects.end();
}

bool ObjectMapNode::is_surface_bound(const std::string& label) const
{
    return std::find(surface_objects.begin(), surface_objects.end(), label) != surface_objects.end();
}

bool ObjectMapNode::is_table_top_object(const std::string& label) const
{
    return std::find(table_top_objects.begin(), table_top_objects.end(), label) != table_top_objects.end();
}

bool ObjectMapNode::is_table_octagon_mode_enabled() const
{
    return table_octagon_refinement_mode != "none";
}

void ObjectMapNode::apply_z_axis_depth_constraints(
    auv_msgs::msg::VisionObject& object_msg,
    const Eigen::Vector3d& filter_position) const
{
    if (!enable_z_axis_locking) {
        object_msg.pose.position.z = filter_position(2);
        return;
    }

    if (is_floor_bound(object_msg.label)) {
        object_msg.pose.position.z = pool_floor_z;
    } else if (is_surface_bound(object_msg.label)) {
        object_msg.pose.position.z = pool_surface_z;
    } else {
        object_msg.pose.position.z = filter_position(2);
    }
}

auv_msgs::msg::VisionObject ObjectMapNode::build_object_message(
    const std::string& label,
    int id,
    const Eigen::Vector3d& position,
    bool has_orientation,
    double theta_z,
    double confidence,
    int frames_since_last_seen) const
{
    auv_msgs::msg::VisionObject object_msg;
    object_msg.header.stamp = frame_collection_time;
    object_msg.header.frame_id = world_frame_id;
    object_msg.label = label;
    object_msg.id = id;
    object_msg.pose.position.x = position(0);
    object_msg.pose.position.y = position(1);
    apply_z_axis_depth_constraints(object_msg, position);

    if (object_sizes_map.count(label)) {
        object_msg.size.x = object_sizes_map.at(label).x();
        object_msg.size.y = object_sizes_map.at(label).y();
        object_msg.size.z = object_sizes_map.at(label).z();
    } else {
        object_msg.size.x = 0.0;
        object_msg.size.y = 0.0;
        object_msg.size.z = 0.0;
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_z);
    object_msg.pose.orientation = tf2::toMsg(q);
    object_msg.has_orientation = has_orientation;
    object_msg.confidence = confidence;
    object_msg.frames_since_last_seen = frames_since_last_seen;
    return object_msg;
}

void ObjectMapNode::down_cam_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    // Skip if no detections
    if (msg->detections.empty()) {
        return;
    }

    double fx, fy, cx, cy;
    {

        fx = down_cam_fx;
        fy = down_cam_fy;
        cx = down_cam_cx;
        cy = down_cam_cy;
    }

    if (fx <= 0.0) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Down camera intrinsics not yet received on camera info topic. Skipping detections.");
        return;
    }

    geometry_msgs::msg::TransformStamped camera_to_world_msg;
    try {
        camera_to_world_msg = tf_buffer_->lookupTransform(
            world_frame_id,
            msg->header.frame_id,
            tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Skipping down cam detection because TF %s -> %s is unavailable: %s",
            world_frame_id.c_str(),
            msg->header.frame_id.c_str(),
            ex.what());
        return;
    }

    tf2::Transform tf_world_camera;
    tf2::fromMsg(camera_to_world_msg.transform, tf_world_camera);
    tf2::Matrix3x3 world_camera_basis = tf_world_camera.getBasis();
    
    Eigen::Vector3d p0(
        camera_to_world_msg.transform.translation.x,
        camera_to_world_msg.transform.translation.y,
        camera_to_world_msg.transform.translation.z);


    std::vector<Eigen::Vector3d> new_measurements;
    std::vector<Eigen::Matrix3d> new_covariances;
    std::vector<std::string> new_classes;
    std::vector<double> new_orientations;
    std::vector<double> new_confidences;

    for (const auto& detection : msg->detections) {
        if (detection.results.empty()) continue;

        // Find best result
        double best_score = -1.0;
        std::string best_class = "";
        for (const auto& result : detection.results) {
            if (result.hypothesis.score > best_score) {
                best_score = result.hypothesis.score;
                best_class = result.hypothesis.class_id;
            }
        }
        
        if (best_class.empty()) continue;
        
        // Center of bounding box
        double u = detection.bbox.center.position.x;
        double v = detection.bbox.center.position.y;
        
        // Ray casting in camera optical frame (accounting for flat port water refraction)
        // NOTE: Because we set z_c = 1.0, this ray is formed in the standard OPTICAL frame 
        // (Z points into the scene). Therefore, the world_camera_basis matrix MUST come 
        // from a "..._optical" TF frame, otherwise the ray will shoot straight forward
        double effective_fx = fx * water_refraction_scale;
        double effective_fy = fy * water_refraction_scale;
        double x_c = (u - cx) / effective_fx;
        double y_c = (v - cy) / effective_fy;
        double z_c = 1.0;
        tf2::Vector3 ray_cam(x_c, y_c, z_c);
        
        // Ray direction in world frame
        tf2::Vector3 ray_world = world_camera_basis * ray_cam;
        
        // Determine the target projection plane based on the object's configured height from the floor
        if (!down_cam_heights_map.count(best_class)) {
            continue; // Skip objects not explicitly configured for down camera projection
        }
        double target_z = pool_floor_z + down_cam_heights_map.at(best_class);
        
        if (std::abs(ray_world.z()) < 1e-6) {
            continue; // Ray is horizontal
        }
        
        double t = (target_z - p0.z()) / ray_world.z();
        if (t <= 0) {
            continue; // Target plane is behind the camera
        }
        
        Eigen::Vector3d pos_world(
            p0.x() + t * ray_world.x(),
            p0.y() + t * ray_world.y(),
            p0.z() + t * ray_world.z());
            
        // Use a generic diagonal covariance for down cam detections
        Eigen::Matrix3d cov_world = Eigen::Matrix3d::Identity() * 0.2;

        new_measurements.push_back(pos_world);
        new_covariances.push_back(cov_world);
        new_classes.push_back(best_class);
        new_orientations.push_back(0.0);
        new_confidences.push_back(best_score);
    }
    
    // Provide persistent positions so the down tracker can re-initialize tracks if they re-enter the FOV
    std::vector<std::pair<std::string, Eigen::Vector3d>> persistent_positions;
    for (const auto& [label, track] : persistent_objects) {
        persistent_positions.push_back({label, track.get_position()});
    }

    // Directly update the independent down camera tracker
    down_tracker.update(
        new_measurements,
        new_covariances,
        new_classes,
        new_orientations,
        new_confidences,
        p0,   // Observer position (camera center)
        true, // Has observer position
        persistent_positions
    );
}

void ObjectMapNode::down_cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{

    down_cam_fx = msg->k[0];
    down_cam_fy = msg->k[4];
    down_cam_cx = msg->k[2];
    down_cam_cy = msg->k[5];
    
    // Unsubscribe from further camera info updates since intrinsics are static
    down_cam_info_subscriber.reset();
    RCLCPP_INFO(this->get_logger(), "Successfully received down camera intrinsics. Unsubscribed from CameraInfo topic.");
}

void ObjectMapNode::detection_callback(const auv_msgs::msg::VisionDetectionFrame::SharedPtr msg)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    frame_collection_time = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    world_frame_id = msg->auv_pose.header.frame_id.empty() ? "pool_link" : msg->auv_pose.header.frame_id;
    const std::string detection_frame_id =
        msg->header.frame_id.empty() ? "zed_left_camera_frame" : msg->header.frame_id;

    geometry_msgs::msg::TransformStamped camera_to_auv_msg;
    try {
        camera_to_auv_msg = tf_buffer_->lookupTransform(
            auv_frame_id,
            detection_frame_id,
            tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Skipping detection frame because TF %s -> %s is unavailable: %s",
            auv_frame_id.c_str(),
            detection_frame_id.c_str(),
            ex.what());
        return;
    }

    // auto t1 = std::chrono::high_resolution_clock::now();

    tf2::Transform tf_world_auv;
    tf2::fromMsg(msg->auv_pose.pose, tf_world_auv);

    tf2::Transform tf_auv_camera;
    tf2::fromMsg(camera_to_auv_msg.transform, tf_auv_camera);

    tf2::Transform tf_world_camera = tf_world_auv * tf_auv_camera;
    tf2::Matrix3x3 world_camera_basis = tf_world_camera.getBasis();

    Eigen::Matrix3d camera_to_world_rotation;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            camera_to_world_rotation(r, c) = world_camera_basis[r][c];
        }
    }

    Eigen::Vector3d observer_position = eigen_from_point(msg->auv_pose.pose.position);
    // Currently always true as auv_pose is bundled in the synchronized detection frame,
    // but kept as a flag for the tracker to support future non-localized updates if
    // a camera does not have an auv_pose.
    bool has_observer_position = true;

    std::vector<Eigen::Vector3d> filtered_measurements;
    std::vector<Eigen::Matrix3d> filtered_covariances;
    std::vector<std::string> filtered_classes;
    std::vector<double> filtered_orientations;
    std::vector<double> filtered_confidences;

    filtered_measurements.reserve(msg->detections.size());
    filtered_covariances.reserve(msg->detections.size());
    filtered_classes.reserve(msg->detections.size());
    filtered_orientations.reserve(msg->detections.size());
    filtered_confidences.reserve(msg->detections.size());

    for (const auto& detection : msg->detections) {
        const std::string& label = detection.label;

        Eigen::Vector3d pos_camera = eigen_from_point(detection.pose_camera.pose.position);
        if (enable_pipe_distance_truncation && (label == "red_pipe" || label == "white_pipe")) {
            if (pos_camera.norm() > max_pipe_distance) {
                continue;
            }
        }

        tf2::Vector3 pos_world_tf = tf_world_camera * tf2::Vector3(
            pos_camera.x(),
            pos_camera.y(),
            pos_camera.z());
        Eigen::Vector3d pos_world(
            pos_world_tf.x(),
            pos_world_tf.y(),
            pos_world_tf.z());

        // Lane boundary filter: discard detections outside the competition lane
        if (enable_lane_boundary) {
            if (pos_world.x() < lane_x_min || pos_world.x() > lane_x_max ||
                pos_world.y() < lane_y_min || pos_world.y() > lane_y_max) {
                continue;
            }
        }

        Eigen::Matrix3d cov_camera = covariance_from_ros_pose(detection.pose_camera.covariance);
        Eigen::Matrix3d cov_world =
            camera_to_world_rotation * cov_camera * camera_to_world_rotation.transpose();
        cov_world += Eigen::Matrix3d::Identity() * 0.3;

        filtered_measurements.push_back(pos_world);
        filtered_covariances.push_back(cov_world);
        filtered_classes.push_back(label);
        filtered_orientations.push_back(0.0);
        filtered_confidences.push_back(detection.confidence);
    }

    // Build persistent positions for the tracker's large-structure proximity check.
    // These are objects that may have been pruned from the tracker (max_age exceeded)
    // but are still held in the node's persistent map.

    // auto t2 = std::chrono::high_resolution_clock::now();

    std::vector<std::pair<std::string, Eigen::Vector3d>> persistent_positions;
    persistent_positions.reserve(persistent_objects.size());
    for (const auto& [label, track] : persistent_objects) {
        persistent_positions.emplace_back(label, track.get_position());
    }

    // Update the independent front camera tracker
    std::vector<Track> front_tracks = front_tracker.update(
        filtered_measurements,
        filtered_covariances,
        filtered_classes,
        filtered_orientations,
        filtered_confidences,
        observer_position,
        has_observer_position,
        persistent_positions);

    // Get the latest tracks from the down camera tracker
    const std::vector<Track>& down_tracks = down_tracker.get_tracks();

    // Combine tracks from both trackers for publishing and persistence logic
    std::vector<Track> all_tracks;
    all_tracks.reserve(front_tracks.size() + down_tracks.size());
    all_tracks.insert(all_tracks.end(), front_tracks.begin(), front_tracks.end());
    all_tracks.insert(all_tracks.end(), down_tracks.begin(), down_tracks.end());

    // auto t3 = std::chrono::high_resolution_clock::now();

    publish_object_map(all_tracks);

    auto t4 = std::chrono::high_resolution_clock::now();

    // auto d_tf    = std::chrono::duration<double, std::milli>(t1 - t0).count();
    // auto d_pre   = std::chrono::duration<double, std::milli>(t2 - t1).count();
    // auto d_track = std::chrono::duration<double, std::milli>(t3 - t2).count();
    // auto d_pub   = std::chrono::duration<double, std::milli>(t4 - t3).count();
    auto d_total = std::chrono::duration<double, std::milli>(t4 - t0).count();

    // rclcpp::Time pipeline_end_time = this->now();
    // rclcpp::Duration pipeline_latency = pipeline_end_time - frame_collection_time;

    // RCLCPP_DEBUG(
    //     this->get_logger(),
    //     "Object map latency: %.3f ms | TF: %.2f Pre: %.2f Track: %.2f Pub: %.2f | Pipe: %.1f ms",
    //     d_total, d_tf, d_pre, d_track, d_pub, pipeline_latency.seconds() * 1000.0);

    RCLCPP_DEBUG(this->get_logger(), "Object map latency: %.3f ms", d_total);
}

void ObjectMapNode::publish_object_map(const std::vector<Track>& tracks)
{
    auv_msgs::msg::VisionObjectArray object_map_msg;
    object_map_msg.header.stamp = frame_collection_time;
    object_map_msg.header.frame_id = world_frame_id;
    std::vector<Track> publish_tracks;

    for (const auto& track : tracks) {
        if (track.state == TrackState::CONFIRMED) {
            if (is_unique_object(track.label)) {
                bool has_saved_orientation = persistent_objects[track.label].has_orientation;
                double saved_theta_z = persistent_objects[track.label].theta_z;

                persistent_objects[track.label] = track;

                if (!track.has_orientation && has_saved_orientation) {
                    persistent_objects[track.label].has_orientation = true;
                    persistent_objects[track.label].theta_z = saved_theta_z;
                }
            } else {
                publish_tracks.push_back(track);
            }
        }
    }

    for (auto& [label, perm_track] : persistent_objects) {
        publish_tracks.push_back(perm_track);
    }
    // Octagon/Table refinement
    // Gets pointers to current table and octagon tracks if they exist
    const Track* table_track = nullptr;
    const Track* octagon_track = nullptr;
    for (const auto& track : publish_tracks) {
        if (track.label == "table") {
            table_track = &track;
        } else if (track.label == "octagon") {
            octagon_track = &track;
        }
    }
        // Set all non-table/octagon tracks for publishing 
    for (const auto& track : publish_tracks) {
        if (is_table_octagon_mode_enabled() &&
            (track.label == "table" || track.label == "octagon")) {
            continue;
        }
        object_map_msg.array.push_back(build_object_message(
            track.label,
            track.id,
            track.get_position(),
            track.has_orientation,
            track.theta_z,
            track.confidence,
            track.age));
    }
    // Refine Octagon
    // Set octagon to table track xy if it exists then apply surface z
    // Then add a ID paired back to table
    if (table_octagon_refinement_mode == "table_primary" && table_track != nullptr) {
        object_map_msg.array.push_back(build_object_message(
            "table",
            table_track->id,
            table_track->get_position(),
            table_track->has_orientation,
            table_track->theta_z,
            table_track->confidence,
            table_track->age));

        Eigen::Vector3d octagon_position = table_track->get_position();
        octagon_position.z() = pool_surface_z;
        object_map_msg.array.push_back(build_object_message(
            "octagon",
            table_track->id + kSyntheticPairIdOffset,
            octagon_position,
            false,
            0.0,
            table_track->confidence,
            table_track->age));
    }
    // Refine Table
    // Set octagon to octagon track xy if it exists then apply floor z
    // Then add a ID paired back to octagon
    if (table_octagon_refinement_mode == "octagon_primary" && octagon_track != nullptr) {
        Eigen::Vector3d octagon_position = octagon_track->get_position();
        octagon_position.z() = pool_surface_z;
        object_map_msg.array.push_back(build_object_message(
            "octagon",
            octagon_track->id,
            octagon_position,
            false,
            0.0,
            octagon_track->confidence,
            octagon_track->age));

        Eigen::Vector3d table_position = octagon_track->get_position();
        table_position.z() = pool_floor_z;
        object_map_msg.array.push_back(build_object_message(
            "table",
            octagon_track->id + kSyntheticPairIdOffset,
            table_position,
            false,
            0.0,
            octagon_track->confidence,
            octagon_track->age));
    }
    // use midpoint of the two instead of setting one to the other
    if (table_octagon_refinement_mode == "midpoint" && (table_track != nullptr || octagon_track != nullptr)) {
        Eigen::Vector2d pair_xy = Eigen::Vector2d::Zero();
        // if both exist take midpoint, otherwise take existing track's xy
        if (table_track != nullptr && octagon_track != nullptr) {
            pair_xy = (xy(table_track->get_position()) + xy(octagon_track->get_position())) / 2.0;
        } else if (table_track != nullptr) {
            pair_xy = xy(table_track->get_position());
        } else {
            pair_xy = xy(octagon_track->get_position());
        }
        // assign table Z to floor
        Eigen::Vector3d table_position =
            table_track != nullptr
                ? table_track->get_position()
                : Eigen::Vector3d(pair_xy.x(), pair_xy.y(), pool_floor_z);
        table_position.x() = pair_xy.x();
        table_position.y() = pair_xy.y();
        // assign octagon Z to surface
        Eigen::Vector3d octagon_position(pair_xy.x(), pair_xy.y(), pool_surface_z);
        // assign table id based on if it depended on octagon
        const int table_id =
            table_track != nullptr
                ? table_track->id
                : octagon_track->id + kSyntheticPairIdOffset;
        // assign octagon id based on if it depended on table
        const int octagon_id =
            octagon_track != nullptr
                ? octagon_track->id
                : table_track->id + kSyntheticPairIdOffset;
        // only table has an orientation component
        const bool table_has_orientation =
            table_track != nullptr && table_track->has_orientation;
        const double table_theta_z =
            table_track != nullptr ? table_track->theta_z : 0.0;
        const double table_confidence =
            table_track != nullptr ? table_track->confidence : octagon_track->confidence;
        const double octagon_confidence =
            octagon_track != nullptr ? octagon_track->confidence : table_track->confidence;
        const int table_age =
            table_track != nullptr ? table_track->age : octagon_track->age;
        const int octagon_age =
            octagon_track != nullptr ? octagon_track->age : table_track->age;

        object_map_msg.array.push_back(build_object_message(
            "table",
            table_id,
            table_position,
            table_has_orientation,
            table_theta_z,
            table_confidence,
            table_age));

        object_map_msg.array.push_back(build_object_message(
            "octagon",
            octagon_id,
            octagon_position,
            false,
            0.0,
            octagon_confidence,
            octagon_age));
    }

    object_map_publisher->publish(object_map_msg);
    rclcpp::Time pipeline_end_time = this->now();
        rclcpp::Duration time_diff = pipeline_end_time - frame_collection_time;
        RCLCPP_DEBUG(
            this->get_logger(),
            "Object map pipeline latency: %.9f seconds",
            time_diff.seconds());
}

