#include <chrono>
#include <string>
#include <map>
#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "auv_msgs/msg/vision_object.hpp"
#include "auv_msgs/msg/vision_object_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#ifdef HAS_ZED_SDK
	#include "zed_detection.hpp"
#endif
#include "object_tracker.hpp"
#include "object.hpp"
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
	// ZED SDK usage
	zed_sdk = this->declare_parameter<bool>("zed_sdk");	
	RCLCPP_INFO(this->get_logger(), "[INIT] zed_sdk param: %s", zed_sdk ? "true" : "false");
	bool sdk_available = false;
// compile time check for ZED SDK see CMakeLists.txt
#ifdef HAS_ZED_SDK
		sdk_available = true;
#endif
	// final decision on using ZED SDK
	zed_sdk &= sdk_available;

	// Subscriber for front cam detections (YOLO from Python node)
	string front_cam_detection_topic = this->declare_parameter<string>("front_cam_detection_topic");
	// Publisher for object map
	string object_map_topic = this->declare_parameter<string>("object_map_topic");
	// Publisher for VIO pose
	// TODO: this should be a subscriber to the DVL pose
	string vio_pose_topic = this->declare_parameter<string>("vio_pose_topic");
	
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
		
	// Object Tracker, used in both ZED and non-ZED modes, so that we maintain a consistent object map output regardless of input source
	object_tracker = ObjectTracker(
		new_object_min_distance_threshold,
		gating_threshold,
		min_hits,
		max_age,
		max_position_jump,
		conf_to_tent_threshold,
		tent_init_buffer,
		enable_gate_midpoint_refinement
	);
	// Publishers
	object_map_publisher = this->create_publisher<auv_msgs::msg::VisionObjectArray>(object_map_topic, 10);

	// Subscriber for world depth (from depth sensor)
	depth_subscriber = this->create_subscription<std_msgs::msg::Float64 >(
		"/sensors/depth/z", 10, std::bind(&ObjectMapNode::depth_callback, this, std::placeholders::_1)
	);
	// Subscribe to 2D detections
	detection_subscriber = this->create_subscription<vision_msgs::msg::Detection2DArray>(
		front_cam_detection_topic, 10, std::bind(&ObjectMapNode::detection_callback, this, std::placeholders::_1)
	);

	if (!zed_sdk)
	{	
		RCLCPP_INFO(this->get_logger(), "ZED SDK not found, using front camera detection topic: %s", front_cam_detection_topic.c_str());
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "ZED SDK found, using ZED camera for object mapping with external detections from: %s", front_cam_detection_topic.c_str());
		


		// --- ZED Parameters ---
		// frame rate for ZED camera
		int frame_rate = this->declare_parameter<int>("frame_rate");
		// confidence threshold for YOLO detections
		float confidence_threshold = this->declare_parameter<float>("confidence_threshold");
		// maximum distance to consider a detection for object mapping
		float max_range_distance_threshold = this->declare_parameter<float>("max_range");
		// use UDP stream for input frames
		bool use_stream = this->declare_parameter<bool>("use_stream");
		// UDP stream IP and port
		string stream_ip = this->declare_parameter<string>("stream_address");
		int stream_port = this->declare_parameter<int>("stream_port");
		// whether run in simulation or real-world (affects ZED SDK settings)
		bool sim = this->declare_parameter<bool>("sim");
		// max depth from ZED camera to consider a new object
		int zed_depth_confidence_threshold = this->declare_parameter<int>("zed_depth_confidence_threshold");
		
		// physical tracking constraints
		enable_gate_top_crop = this->declare_parameter<bool>("enable_gate_top_crop");
		gate_top_crop_ratio = this->declare_parameter<double>("gate_top_crop_ratio");
		enable_z_axis_locking = this->declare_parameter<bool>("enable_z_axis_locking");
		enable_octagon_xy_inheritance = this->declare_parameter<bool>("enable_octagon_xy_inheritance");
		pool_floor_z = this->declare_parameter<double>("pool_floor_z");
		pool_surface_z = this->declare_parameter<double>("pool_surface_z");
		unique_objects = this->declare_parameter<std::vector<std::string>>("unique_objects");
		floor_objects = this->declare_parameter<std::vector<std::string>>("floor_objects");
		surface_objects = this->declare_parameter<std::vector<std::string>>("surface_objects");
		max_pipe_distance = this->declare_parameter<double>("max_pipe_distance");

		RCLCPP_INFO(this->get_logger(), "[INIT] ZED params: sim=%s, use_stream=%s, stream=%s:%d, frame_rate=%d",
			sim ? "true" : "false", use_stream ? "true" : "false", stream_ip.c_str(), stream_port, frame_rate);
		
#ifdef HAS_ZED_SDK
		ZEDCameraModel camera_model = sim ? ZEDCameraModel::ZEDX : ZEDCameraModel::ZED2i;
		RCLCPP_INFO(this->get_logger(), "[INIT] Creating ZEDDetection with camera model: %s", sim ? "ZEDX" : "ZED2i");
		try {
			zed_detector = std::make_unique<ZEDDetection>(
				frame_rate,
				confidence_threshold,
				max_range_distance_threshold,
				use_stream,
				stream_ip,
				stream_port,
				camera_model,
				zed_depth_confidence_threshold,
				// add callbacks to use rclcpp logging
				[this] (const string& msg) { RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str()); },
				[this](const string& msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); },
				[this](const string& msg) { RCLCPP_WARN(this->get_logger(), "%s", msg.c_str()); },
				[this](const string& msg, int throttle_duration_ms) { RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), throttle_duration_ms, "%s", msg.c_str()); },
				[this](const string& msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); },
				[this](const string& msg) { RCLCPP_FATAL(this->get_logger(), "%s", msg.c_str()); }
			);
			RCLCPP_INFO(this->get_logger(), "[INIT] ZEDDetection created successfully");
		} catch (const std::exception& e) {
			RCLCPP_FATAL(this->get_logger(), "[INIT] ZEDDetection creation failed: %s", e.what());
			throw;
		}
#endif
	}
	// Publisher for VIO pose from ZED SDK
	pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(vio_pose_topic, 10);
	RCLCPP_INFO(this->get_logger(), "Object Map Node has been initialized");
	if (zed_sdk)
	{
		RCLCPP_INFO(this->get_logger(), "Using ZED SDK for object mapping.");
    }
	else
	{
		RCLCPP_INFO(this->get_logger(), "Using front camera detections for object mapping.");
	}
}
private:
	std::map<std::string, Track> persistent_objects;
	bool enable_gate_top_crop;
	bool enable_z_axis_locking;
	bool enable_octagon_xy_inheritance;
	double pool_floor_z;
	double pool_surface_z;
	double gate_top_crop_ratio;
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

	void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
	{
#ifdef HAS_ZED_SDK
		if(!zed_detector) {
			RCLCPP_WARN(this->get_logger(), "[CB] zed_detector is null, skipping");
			return;
		}
		frame_collection_time = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
		std::vector<sl::CustomBoxObjectData> zed_detections = extract_ZED_detections(msg);
		// Convert ROS message to ZED SDK CustomBoxObjectData
		zed_detector->process_detections(zed_detections);
		const auto [measurements,covariances,classes,orientations,confidences] = zed_detector->GetDetections();
		
		std::vector<Eigen::Vector3d> filtered_measurements;
		std::vector<Eigen::Matrix3d> filtered_covariances;
		std::vector<std::string> filtered_classes;
		std::vector<double> filtered_orientations;
		std::vector<double> filtered_confidences;

		for (size_t i = 0; i < measurements.size(); ++i) {
			// Filter: Skip pipes detected further than our distance threshold
			if (classes[i] == "red_pipe" || classes[i] == "white_pipe") {
				// We don't have the raw camera-frame position here easily, but the world 
				// position's distance from the camera pose is roughly the same.
				// For exact equivalence with Python, we can just check the norm from the camera.
				auto cam_pose = zed_detector->GetCameraPose();
				Eigen::Vector3d cam_trans = std::get<0>(cam_pose);
				double dist_from_robot = (measurements[i] - cam_trans).norm();
				if (dist_from_robot > max_pipe_distance) {
					continue;
				}
			}
			filtered_measurements.push_back(measurements[i]);
			filtered_covariances.push_back(covariances[i]);
			filtered_classes.push_back(classes[i]);
			filtered_orientations.push_back(orientations[i]);
			filtered_confidences.push_back(confidences[i]);
		}

		// The classes returned by `GetDetections` are strings derived from int label inside `ZEDDetection`.
		// So passing the correct int label is crucial.
		
		std::vector<Track> all_tracks = object_tracker.update(
			filtered_measurements, 
			filtered_covariances, 
			filtered_classes, 
			filtered_orientations, 
			filtered_confidences
		);
		publish_pose(zed_detector->GetCameraPose());
		publish_object_map(all_tracks);
#endif
	}

	void depth_callback(const std_msgs::msg::Float64::SharedPtr msg)
	{
#ifdef HAS_ZED_SDK
		if(zed_detector)
		{
			zed_detector->UpdateSensorDepth(msg->data);
		}
#endif
	}

	void front_cam_callback(const auv_msgs::msg::VisionObjectArray::SharedPtr msg)
	{
		(void)msg;
		// Fallback is a no-op for now
		// we can add determining depth and publishing if we ever choose to not use ZED_SDK 
		// this is added for compilation on both nvidia and non-nvidia machines 
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

	void publish_pose(const std::tuple<Eigen::Vector3d,Eigen::Vector4d>& pose)
	{
		geometry_msgs::msg::PoseStamped pose_msg;
		pose_msg.header.stamp = frame_collection_time;
		pose_msg.header.frame_id = "world";
		pose_msg.pose.position.x = std::get<0>(pose)(0);
		pose_msg.pose.position.y = std::get<0>(pose)(1);
		pose_msg.pose.position.z = std::get<0>(pose)(2);
		pose_msg.pose.orientation.w = std::get<1>(pose)(0);
		pose_msg.pose.orientation.x = std::get<1>(pose)(1);
		pose_msg.pose.orientation.y = std::get<1>(pose)(2);
		pose_msg.pose.orientation.z = std::get<1>(pose)(3);
		pose_publisher->publish(pose_msg);
	}
#ifdef HAS_ZED_SDK
	std::vector<sl::CustomBoxObjectData> extract_ZED_detections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
	{
		std::vector<sl::CustomBoxObjectData> zed_detections;
		
		for(const auto& detection : msg->detections) {
			sl::CustomBoxObjectData box;
			box.unique_object_id = sl::generate_unique_id();
            
            box.label = -1; // Default to unknown

			// Label & Confidence (taking top hypothesis)
			if (!detection.results.empty()) {
				box.probability = detection.results[0].hypothesis.score;
                
                std::string label = detection.results[0].hypothesis.class_id;
                auto it = std::find(ID_TO_LABEL.begin(), ID_TO_LABEL.end(), label);
                if (it != ID_TO_LABEL.end()) {
                    box.label = std::distance(ID_TO_LABEL.begin(), it);
                }
			}

            if (box.label != -1) {
			    box.is_static = true;
			    
			    // 2D Bounding Box
			    float cx = detection.bbox.center.position.x;
			    float cy = detection.bbox.center.position.y;
			    float w = detection.bbox.size_x;
			    float h = detection.bbox.size_y;
			    
			    // --- Post-Processing Physical Constraints ---
			    // If the object is a gate, only feed the ZED SDK the top portion of the bounding box.
			    // The gate's legs extend deep into noisy acoustic territory causing bad depth estimations.
			    if (enable_gate_top_crop && ID_TO_LABEL[box.label] == "gate") {
			        float original_top = cy - h / 2.0f;
			        h = h * gate_top_crop_ratio;
			        cy = original_top + h / 2.0f;
			    }

			    float left = cx - w / 2.0f;
			    float top = cy - h / 2.0f;
			    float right = cx + w / 2.0f;
			    float bottom = cy + h / 2.0f;
			    
			    std::vector<sl::uint2> bbox_2d(4);
			    bbox_2d[0] = sl::uint2(static_cast<unsigned int>(left), static_cast<unsigned int>(top));     // Top-Left
			    bbox_2d[1] = sl::uint2(static_cast<unsigned int>(right), static_cast<unsigned int>(top));    // Top-Right
			    bbox_2d[2] = sl::uint2(static_cast<unsigned int>(right), static_cast<unsigned int>(bottom)); // Bottom-Right
			    bbox_2d[3] = sl::uint2(static_cast<unsigned int>(left), static_cast<unsigned int>(bottom));  // Bottom-Left
			    box.bounding_box_2d = bbox_2d;

			    zed_detections.push_back(box);
            }
		}
		return zed_detections;
	}
#endif
#ifdef HAS_ZED_SDK
	std::unique_ptr<ZEDDetection> zed_detector;
#endif

	ObjectTracker object_tracker;

	bool zed_sdk;
	rclcpp::Time frame_collection_time;
	rclcpp::Publisher<auv_msgs::msg::VisionObjectArray>::SharedPtr object_map_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_subscriber;
	// if zed SDK not in use
	rclcpp::Subscription<auv_msgs::msg::VisionObjectArray>::SharedPtr front_cam_subscriber;
	rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_subscriber;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectMapNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
