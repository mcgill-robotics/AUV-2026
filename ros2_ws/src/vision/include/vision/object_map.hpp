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

    void publish_object_map(const std::vector<Track>& tracks);

    std::map<std::string, Track> persistent_objects;
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
    std::string auv_frame_id;
    std::string world_frame_id = "pool_link";

    ObjectTracker object_tracker;
    rclcpp::Time frame_collection_time;
    rclcpp::Publisher<auv_msgs::msg::VisionObjectArray>::SharedPtr object_map_publisher;
    rclcpp::Subscription<auv_msgs::msg::VisionDetectionFrame>::SharedPtr detection_subscriber;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};