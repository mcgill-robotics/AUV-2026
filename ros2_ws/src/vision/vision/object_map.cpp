#include <chrono>
#include <string>
#include <map>
#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "auv_msgs/msg/vision_object.hpp"
#include "auv_msgs/msg/vision_object_array.hpp"



#include "vision_msgs/msg/detection3_d_array.hpp"
#include "object_tracker.hpp"
#include <algorithm>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
// alias for milliseconds with double precision
using double_ms = std::chrono::duration<double, std::milli>;

class ObjectMapNode : public rclcpp::Node
{
public:
    ObjectMapNode() : Node("object_map_node")
    {
	RCLCPP_INFO(this->get_logger(), "[INIT] ObjectMapNode constructor started");
	// adding "this" boilerplate for methods called from rclcpp Node base class
	// Removing ALL default parameters to ensure we explicitly set everything we need for both ZED and non-ZED modes, and to avoid accidentally relying on defaults that might not be what we expect (these were originally kept for debugging)
	// ZED SDK depth extraction and pose publishing are now handled by front_cam_object_detection python node
	// Subscriber for front cam detections (YOLO from Python node)
	string front_cam_detection_topic = this->declare_parameter<string>("front_cam_detection_topic");
	// Publisher for object map
	string object_map_topic = this->declare_parameter<string>("object_map_topic");
	
	// tracker tuning parameters used in both ZED and non-ZED fallback
	// minimum distance from existing tracks for a new detection to be considered a new object rather than a new measurement of an existing object
	float new_object_min_distance_threshold = this->declare_parameter<float>("new_object_min_distance_threshold");
	// gating threshold for associating detections to existing tracks (in Mahalanobis distance)
	double gating_threshold  = this->declare_parameter<double>("gating_threshold");
	// number of consecutive detections needed to confirm a new track
	int min_hits = this->declare_parameter<int>("min_hits");
	// number of consecutive missed detections before a track is deleted
	int max_age = this->declare_parameter<int>("max_age");
	// maximum allowed jump in position for an object between consecutive detections
	double max_position_jump = this->declare_parameter<double>("max_position_jump");
	// confidence threshold for associating a detection to an existing track rather than initializing a new track
	int conf_to_tent_threshold = this->declare_parameter<int>("conf_to_tent_threshold");
	// number of consecutive detections below confidence threshold before a track is deleted
	int tent_init_buffer = this->declare_parameter<int>("tent_init_buffer");

	bool enable_gate_midpoint_refinement = this->declare_parameter<bool>("enable_gate_midpoint_refinement");
		
	// Fetch dynamic labels and max limits
	this->class_labels = this->declare_parameter<std::vector<std::string>>("class_labels");
	std::vector<std::string> max_per_class_labels = this->declare_parameter<std::vector<std::string>>("max_per_class_labels");
	std::vector<int64_t> max_per_class_values = this->declare_parameter<std::vector<int64_t>>("max_per_class_values");

	std::unordered_map<std::string, int> max_per_class_map;
	for (size_t i = 0; i < max_per_class_labels.size() && i < max_per_class_values.size(); ++i) {
		max_per_class_map[max_per_class_labels[i]] = static_cast<int>(max_per_class_values[i]);
	}

	// Object Tracker, used in both ZED and non-ZED modes, so that we maintain a consistent object map output regardless of input source
	object_tracker = ObjectTracker(
        max_per_class_map,
		new_object_min_distance_threshold,
		gating_threshold,
		min_hits,
		max_age,
		max_position_jump,
		conf_to_tent_threshold,
		tent_init_buffer,
		enable_gate_midpoint_refinement
	);
	// physical tracking constraints
	enable_z_axis_locking = this->declare_parameter<bool>("enable_z_axis_locking");
	enable_octagon_xy_inheritance = this->declare_parameter<bool>("enable_octagon_xy_inheritance");
	pool_floor_z = this->declare_parameter<double>("pool_floor_z");
	pool_surface_z = this->declare_parameter<double>("pool_surface_z");
	unique_objects = this->declare_parameter<std::vector<std::string>>("unique_objects");
	floor_objects = this->declare_parameter<std::vector<std::string>>("floor_objects");
	surface_objects = this->declare_parameter<std::vector<std::string>>("surface_objects");
	max_pipe_distance = this->declare_parameter<double>("max_pipe_distance");

	// Publishers
	object_map_publisher = this->create_publisher<auv_msgs::msg::VisionObjectArray>(object_map_topic, 10);

	// Subscribe to 3D detections from Python node
	detection_subscriber = this->create_subscription<vision_msgs::msg::Detection3DArray>(
		front_cam_detection_topic, 10, std::bind(&ObjectMapNode::detection_callback, this, std::placeholders::_1)
	);

	RCLCPP_INFO(this->get_logger(), "Using generic 3D camera detections for object mapping.");
}
private:
	std::map<std::string, Track> persistent_objects;
	bool enable_z_axis_locking;
	bool enable_octagon_xy_inheritance;
	double pool_floor_z;
	double pool_surface_z;
	double max_pipe_distance;
	std::vector<std::string> unique_objects;
	std::vector<std::string> floor_objects;
	std::vector<std::string> surface_objects;

	bool is_unique_object(const std::string& label) const {
		return std::find(unique_objects.begin(), unique_objects.end(), label) != unique_objects.end();
	}

	bool is_floor_bound(const std::string& label) const {
		return std::find(floor_objects.begin(), floor_objects.end(), label) != floor_objects.end();
	}

	bool is_surface_bound(const std::string& label) const {
		return std::find(surface_objects.begin(), surface_objects.end(), label) != surface_objects.end();
	}

	// --- Post-Processing Physical Constraints ---
	void apply_z_axis_depth_constraints(auv_msgs::msg::VisionObject& object_msg, const Eigen::Vector3d& filter_position) const {
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

	void detection_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
	{
		frame_collection_time = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
		
		std::vector<Eigen::Vector3d> filtered_measurements;
		std::vector<Eigen::Matrix3d> filtered_covariances;
		std::vector<std::string> filtered_classes;
		std::vector<double> filtered_orientations;
		std::vector<double> filtered_confidences;

		for (const auto& detection : msg->detections) {
			if (detection.results.empty()) continue;

			const auto& hypothesis = detection.results[0];
			std::string label = hypothesis.hypothesis.class_id;
			
			Eigen::Vector3d pos(hypothesis.pose.pose.position.x, 
			                    hypothesis.pose.pose.position.y, 
			                    hypothesis.pose.pose.position.z);

			// Camera translation is packed in bbox.size by the Python node
			Eigen::Vector3d cam_trans(detection.bbox.size.x,
			                         detection.bbox.size.y,
			                         detection.bbox.size.z);

			// Filter: Skip pipes detected further than our distance threshold
			// Use distance from AUV (camera), not from world origin
			if (label == "red_pipe" || label == "white_pipe") {
				double dist_from_robot = (pos - cam_trans).norm();
				if (dist_from_robot > max_pipe_distance) {
					continue;
				}
			}

			// Covariance: already rotated to world frame by the Python node
			Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.1;
			if (hypothesis.pose.covariance.size() == 36) {
				for (int r = 0; r < 3; ++r) {
					for (int c = 0; c < 3; ++c) {
						cov(r, c) = hypothesis.pose.covariance[r * 6 + c];
					}
				}
				// Increased baseline measurement noise to enforce higher Kalman Filter inertia
				cov += Eigen::Matrix3d::Identity() * 0.3;
			}
			
			filtered_measurements.push_back(pos);
			filtered_covariances.push_back(cov);
			filtered_classes.push_back(label);
			filtered_orientations.push_back(0.0);
			filtered_confidences.push_back(hypothesis.hypothesis.score);
		}

		std::vector<Track> all_tracks = object_tracker.update(
			filtered_measurements, 
			filtered_covariances, 
			filtered_classes, 
			filtered_orientations, 
			filtered_confidences
		);

		publish_object_map(all_tracks);
	}


	void publish_object_map(const std::vector<Track>& tracks)
	{
		auv_msgs::msg::VisionObjectArray object_map_msg;
		object_map_msg.header.stamp = frame_collection_time;
		object_map_msg.header.frame_id = "map";

		std::vector<Track> publish_tracks;

		// --- Process Active Tracks ---
		for (const auto& track : tracks) {
			if (track.state == TrackState::CONFIRMED) {
				if (is_unique_object(track.label)) {
					// Update global memory for unique items
					bool has_saved_orientation = persistent_objects[track.label].has_orientation;
					double saved_theta_z = persistent_objects[track.label].theta_z;

					persistent_objects[track.label] = track;

					// Preserve the saved orientation if the new track doesn't have one
					if (!track.has_orientation && has_saved_orientation) {
						persistent_objects[track.label].has_orientation = true;
						persistent_objects[track.label].theta_z = saved_theta_z;
					}
				} else {
					// Add regular confirmed tracks to the publish list
					publish_tracks.push_back(track);
				}
			}
		}

		// Add all persistent unique tracks
		for (auto& [label, perm_track] : persistent_objects) {
			if (enable_octagon_xy_inheritance && label == "octagon") {
				continue; // Completely ignore real octagon tracks
			}
			publish_tracks.push_back(perm_track);
		}

		// for each track, publish as VisionObject
		for (const auto& track : publish_tracks)
		{
			if (enable_octagon_xy_inheritance && track.label == "octagon") {
				continue; // Ignore any real active non-unique octagon tracks (if any)
			}

			auv_msgs::msg::VisionObject object_msg;
			object_msg.header.stamp = frame_collection_time;
			object_msg.header.frame_id = "map";
			
			object_msg.label = track.label;
			object_msg.id = track.id;
			Eigen::Vector3d position = track.get_position();
			
			// Position
			object_msg.pose.position.x = position(0);
			object_msg.pose.position.y = position(1);

			// --- Post-Processing Physical Constraints ---
			apply_z_axis_depth_constraints(object_msg, position);

			// Orientation
			tf2::Quaternion q;
			q.setRPY(0.0, 0.0, track.theta_z);
			object_msg.pose.orientation = tf2::toMsg(q);
			object_msg.has_orientation = track.has_orientation;

			object_msg.confidence = track.confidence;
			object_map_msg.array.push_back(object_msg);

			// --- Post-Processing Physical Constraints ---
			// The octagon always sits directly beneath the table.
			// Vision on the octagon is unreliable due to water surface distortion/reflections.
			// Thus, we inherit the reliable 2D position of the table by generating a synthetic octagon.
			if (enable_octagon_xy_inheritance && track.label == "table") {
				auv_msgs::msg::VisionObject octagon_msg;
				octagon_msg.header.stamp = frame_collection_time;
				octagon_msg.header.frame_id = "map";
				
				octagon_msg.label = "octagon";
				octagon_msg.id = track.id + 1000; // Create unique synthetic ID
				
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
		RCLCPP_INFO(this->get_logger(), "Object map pipeline latency: %.9f seconds", time_diff.seconds());
	}

	ObjectTracker object_tracker;
    std::vector<std::string> class_labels;

	rclcpp::Time frame_collection_time;
	rclcpp::Publisher<auv_msgs::msg::VisionObjectArray>::SharedPtr object_map_publisher;
	rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_subscriber;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectMapNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
