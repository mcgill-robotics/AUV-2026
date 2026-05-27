#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "auv_msgs/msg/vision_detection_frame.hpp"
#include "auv_msgs/msg/vision_object.hpp"
#include "auv_msgs/msg/vision_object_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <mutex>
#include "object_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ObjectMapNode : public rclcpp::Node
{
public:
    ObjectMapNode();

private:
    static constexpr int kSyntheticPairIdOffset = 1000;

    static Eigen::Vector3d eigen_from_point(const geometry_msgs::msg::Point& point);

    static Eigen::Vector2d xy(const Eigen::Vector3d& position);

    static Eigen::Matrix3d covariance_from_ros_pose(const std::array<double, 36>& covariance);

    bool is_unique_object(const std::string& label) const;

    bool is_floor_bound(const std::string& label) const;

    bool is_surface_bound(const std::string& label) const;

    bool is_table_top_object(const std::string& label) const;

    bool is_table_octagon_mode_enabled() const;

    void apply_z_axis_depth_constraints(
        auv_msgs::msg::VisionObject& object_msg,
        const Eigen::Vector3d& filter_position) const;

    auv_msgs::msg::VisionObject build_object_message(
        const std::string& label,
        int id,
        const Eigen::Vector3d& position,
        bool has_orientation,
        double theta_z,
        double confidence,
        int frames_since_last_seen) const;

    void detection_callback(const auv_msgs::msg::VisionDetectionFrame::SharedPtr msg);
    void down_cam_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void down_cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void publish_object_map(const std::vector<Track>& tracks);

    std::map<std::string, Track> persistent_objects;
    std::unordered_map<std::string, Eigen::Vector3d> object_sizes_map;
    bool enable_z_axis_locking;
    std::string table_octagon_refinement_mode;
    double pool_floor_z;
    double pool_surface_z;
    double max_pipe_distance;
    bool enable_pipe_distance_truncation;
    bool enable_lane_boundary;
    double lane_x_min;
    double lane_x_max;
    double lane_y_min;
    double lane_y_max;
    std::vector<std::string> unique_objects;
    std::vector<std::string> floor_objects;
    std::vector<std::string> surface_objects;
    std::vector<std::string> table_top_objects;
    std::string auv_frame_id;
    std::string world_frame_id = "pool_link";

    ObjectTracker object_tracker;
    rclcpp::Time frame_collection_time;
    rclcpp::Publisher<auv_msgs::msg::VisionObjectArray>::SharedPtr object_map_publisher;
    rclcpp::Subscription<auv_msgs::msg::VisionDetectionFrame>::SharedPtr detection_subscriber;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr down_cam_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr down_cam_info_subscriber;
    
    // Down camera intrinsics
    double down_cam_fx;
    double down_cam_fy;
    double down_cam_cx;
    double down_cam_cy;
    double water_refraction_scale;

    // Thread-safe buffer for down-cam projected 3D measurements
    std::mutex down_cam_mutex;
    double table_z;
    std::vector<Eigen::Vector3d> down_cam_measurements;
    std::vector<Eigen::Matrix3d> down_cam_covariances;
    std::vector<std::string> down_cam_classes;
    std::vector<double> down_cam_orientations;
    std::vector<double> down_cam_confidences;
};