#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <string>
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

using namespace std;

class ObjectMapNode : public rclcpp::Node
{
public:
    ObjectMapNode() : Node("object_map_node")
    {
        RCLCPP_INFO(this->get_logger(), "[INIT] ObjectMapNode constructor started");

        string front_cam_detection_frame_topic =
            this->declare_parameter<string>("front_cam_detection_frame_topic");
        string object_map_topic = this->declare_parameter<string>("object_map_topic");
        auv_frame_id = this->declare_parameter<string>("auv_frame_id", "auv_link");

        float new_object_min_distance_threshold =
            this->declare_parameter<float>("new_object_min_distance_threshold");
        float min_large_structure_separation =
            this->declare_parameter<float>("min_large_structure_separation_m");
        float min_large_structure_pipe_separation =
            this->declare_parameter<float>("min_large_structure_pipe_separation_m");
        double gating_threshold = this->declare_parameter<double>("gating_threshold");
        int min_hits = this->declare_parameter<int>("min_hits");
        int max_age = this->declare_parameter<int>("max_age");
        double max_position_jump = this->declare_parameter<double>("max_position_jump");
        int conf_to_tent_threshold = this->declare_parameter<int>("conf_to_tent_threshold");
        int tent_init_buffer = this->declare_parameter<int>("tent_init_buffer");

        bool enable_gate_midpoint_refinement =
            this->declare_parameter<bool>("enable_gate_midpoint_refinement");
        bool enable_board_icon_refinement =
            this->declare_parameter<bool>("enable_board_icon_refinement");

        std::vector<std::string> large_structure_labels =
            this->declare_parameter<std::vector<std::string>>("large_structure_labels");
        std::vector<std::string> pipe_labels =
            this->declare_parameter<std::vector<std::string>>("pipe_labels");
        std::vector<std::string> max_per_class_labels =
            this->declare_parameter<std::vector<std::string>>("max_per_class_labels");
        std::vector<int64_t> max_per_class_values =
            this->declare_parameter<std::vector<int64_t>>("max_per_class_values");

        std::unordered_map<std::string, int> max_per_class_map;
        for (size_t i = 0; i < max_per_class_labels.size() && i < max_per_class_values.size(); ++i) {
            max_per_class_map[max_per_class_labels[i]] = static_cast<int>(max_per_class_values[i]);
        }

        object_tracker = ObjectTracker(
            max_per_class_map,
            large_structure_labels,
            pipe_labels,
            new_object_min_distance_threshold,
            min_large_structure_separation,
            min_large_structure_pipe_separation,
            gating_threshold,
            min_hits,
            max_age,
            max_position_jump,
            conf_to_tent_threshold,
            tent_init_buffer,
            enable_gate_midpoint_refinement,
            enable_board_icon_refinement
        );

        enable_z_axis_locking = this->declare_parameter<bool>("enable_z_axis_locking");
        enable_octagon_xy_inheritance = this->declare_parameter<bool>("enable_octagon_xy_inheritance");
        pool_floor_z = this->declare_parameter<double>("pool_floor_z");
        pool_surface_z = this->declare_parameter<double>("pool_surface_z");
        unique_objects = this->declare_parameter<std::vector<std::string>>("unique_objects");
        floor_objects = this->declare_parameter<std::vector<std::string>>("floor_objects");
        surface_objects = this->declare_parameter<std::vector<std::string>>("surface_objects");
        max_pipe_distance = this->declare_parameter<double>("max_pipe_distance");

        object_map_publisher =
            this->create_publisher<auv_msgs::msg::VisionObjectArray>(object_map_topic, 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        detection_subscriber =
            this->create_subscription<auv_msgs::msg::VisionDetectionFrame>(
                front_cam_detection_frame_topic,
                10,
                std::bind(&ObjectMapNode::detection_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Using synchronized front-camera detection frames for object mapping.");
    }

private:
    static Eigen::Vector3d eigen_from_point(const geometry_msgs::msg::Point& point)
    {
        return Eigen::Vector3d(point.x, point.y, point.z);
    }

    static Eigen::Matrix3d covariance_from_ros_pose(const std::array<double, 36>& covariance)
    {
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.1;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                cov(r, c) = covariance[r * 6 + c];
            }
        }
        return cov;
    }

    bool is_unique_object(const std::string& label) const
    {
        return std::find(unique_objects.begin(), unique_objects.end(), label) != unique_objects.end();
    }

    bool is_floor_bound(const std::string& label) const
    {
        return std::find(floor_objects.begin(), floor_objects.end(), label) != floor_objects.end();
    }

    bool is_surface_bound(const std::string& label) const
    {
        return std::find(surface_objects.begin(), surface_objects.end(), label) != surface_objects.end();
    }

    void apply_z_axis_depth_constraints(
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

    void detection_callback(const auv_msgs::msg::VisionDetectionFrame::SharedPtr msg)
    {
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
            if (label == "red_pipe" || label == "white_pipe") {
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

        std::vector<Track> all_tracks = object_tracker.update(
            filtered_measurements,
            filtered_covariances,
            filtered_classes,
            filtered_orientations,
            filtered_confidences,
            observer_position,
            has_observer_position);

        publish_object_map(all_tracks);
    }

    void publish_object_map(const std::vector<Track>& tracks)
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
            if (enable_octagon_xy_inheritance && label == "octagon") {
                continue;
            }
            publish_tracks.push_back(perm_track);
        }

        for (const auto& track : publish_tracks) {
            if (enable_octagon_xy_inheritance && track.label == "octagon") {
                continue;
            }

            auv_msgs::msg::VisionObject object_msg;
            object_msg.header.stamp = frame_collection_time;
            object_msg.header.frame_id = world_frame_id;

            object_msg.label = track.label;
            object_msg.id = track.id;
            Eigen::Vector3d position = track.get_position();

            object_msg.pose.position.x = position(0);
            object_msg.pose.position.y = position(1);

            apply_z_axis_depth_constraints(object_msg, position);

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, track.theta_z);
            object_msg.pose.orientation = tf2::toMsg(q);
            object_msg.has_orientation = track.has_orientation;

            object_msg.confidence = track.confidence;
            object_map_msg.array.push_back(object_msg);

            if (enable_octagon_xy_inheritance && track.label == "table") {
                auv_msgs::msg::VisionObject octagon_msg;
                octagon_msg.header.stamp = frame_collection_time;
                octagon_msg.header.frame_id = world_frame_id;

                octagon_msg.label = "octagon";
                octagon_msg.id = track.id + 1000;

                octagon_msg.pose.position.x = position(0);
                octagon_msg.pose.position.y = position(1);
                octagon_msg.pose.position.z = pool_surface_z;

                tf2::Quaternion q_oct;
                q_oct.setRPY(0.0, 0.0, 0.0);
                octagon_msg.pose.orientation = tf2::toMsg(q_oct);
                octagon_msg.has_orientation = false;

                octagon_msg.confidence = track.confidence;
                object_map_msg.array.push_back(octagon_msg);
            }
        }

        object_map_publisher->publish(object_map_msg);
        rclcpp::Time pipeline_end_time = this->now();
        rclcpp::Duration time_diff = pipeline_end_time - frame_collection_time;
        RCLCPP_INFO(
            this->get_logger(),
            "Object map pipeline latency: %.9f seconds",
            time_diff.seconds());
    }

    std::map<std::string, Track> persistent_objects;
    bool enable_z_axis_locking;
    bool enable_octagon_xy_inheritance;
    double pool_floor_z;
    double pool_surface_z;
    double max_pipe_distance;
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
