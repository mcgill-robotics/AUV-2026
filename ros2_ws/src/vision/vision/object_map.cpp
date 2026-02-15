#include <chrono>
#include <string>
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

using namespace std;
// alias for milliseconds with double precision
using double_ms = std::chrono::duration<double, std::milli>;

class ObjectMapNode : public rclcpp::Node
{
public:
    ObjectMapNode() : Node("object_map_node")
    {
	// adding "this" boilerplate for methods called from rclcpp Node base class
	// ZED SDK usage
	this->declare_parameter<bool>("zed_sdk", true);
	// minimum distance to consider a detection as a new object
	this->declare_parameter("new_object_min_distance_threshold", 0.5);
	
	// only needed in constructor
	float new_object_distance_threshold;
	this->get_parameter("new_object_min_distance_threshold", new_object_distance_threshold);
	this->get_parameter("zed_sdk", zed_sdk);
	bool sdk_available = false;
// compile time check for ZED SDK see CMakeLists.txt
#ifdef HAS_ZED_SDK
		sdk_available = true;
#endif
	// final decision on using ZED SDK
	zed_sdk &= sdk_available;

	// Subscriber for front cam detections (YOLO from Python node)
	this->declare_parameter<string>("front_cam_detection_topic");
	this->declare_parameter<string>("object_map_topic");
	this->declare_parameter<string>("vio_pose_topic");

	string front_cam_detection_topic;
	string object_map_topic;
	string vio_pose_topic;

	this->get_parameter("front_cam_detection_topic", front_cam_detection_topic);
	this->get_parameter("object_map_topic", object_map_topic);
	this->get_parameter("vio_pose_topic", vio_pose_topic);

	if (!zed_sdk)
	{	
		RCLCPP_INFO(this->get_logger(), "ZED SDK not found, using front camera detection topic: %s", front_cam_detection_topic.c_str());
		front_cam_subscriber = this->create_subscription<auv_msgs::msg::VisionObjectArray>(
			"/vision/front_cam/object_detection_legacy", 10, std::bind(&ObjectMapNode::front_cam_callback, this, std::placeholders::_1)
		);
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "ZED SDK found, using ZED camera for object mapping with external detections from: %s", front_cam_detection_topic.c_str());
		
		// Subscribe to 2D detections
		detection_subscriber = this->create_subscription<vision_msgs::msg::Detection2DArray>(
			front_cam_detection_topic, 10, std::bind(&ObjectMapNode::detection_callback, this, std::placeholders::_1)
		);

		// ZED Parameters
		this->declare_parameter<int>("frame_rate");
		this->declare_parameter<string>("confidence_threshold");
		this->declare_parameter<float>("max_range");
		// use UDP stream for input frames
		this->declare_parameter<bool>("use_stream");
		// UDP stream IP and port
		this->declare_parameter<string>("stream_ip", "127.0.0.1");
		this->declare_parameter<int>("stream_port");
		// show YOLO bounding boxes on output frames
		this->declare_parameter<bool>("show_detections");
		// enable heavy debug table logs
		this->declare_parameter<bool>("debug_logs");

	
		// get parameters
		int frame_rate;
		float confidence_threshold;
		float max_range_distance_threshold;
		bool use_stream;
		string stream_ip;
		int stream_port;
		bool show_detections;
		bool debug_logs;

		this->get_parameter("frame_rate", frame_rate);
		this->get_parameter("confidence_threshold", confidence_threshold);
		this->get_parameter("max_range", max_range_distance_threshold);
		this->get_parameter("use_stream", use_stream);
		this->get_parameter("stream_ip", stream_ip);
		this->get_parameter("stream_port", stream_port);
		this->get_parameter("show_detections", show_detections);
		this->get_parameter("debug_logs", debug_logs);
		
#ifdef HAS_ZED_SDK
		zed_detector = std::make_unique<ZEDDetection>(
			frame_rate,
			confidence_threshold,
			max_range_distance_threshold,
			use_stream,
			stream_ip,
			stream_port,
			show_detections,
			debug_logs,
			// add callbacks to use rclcpp logging
			[this](const string& msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); },
			[this](const string& msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); },
			[this](const string& msg) { RCLCPP_WARN(this->get_logger(), "%s", msg.c_str()); },
			[this](const string& msg, int throttle_duration_ms) { RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), throttle_duration_ms, "%s", msg.c_str()); }
		);
#endif
}
	// Object Tracker
	object_tracker = ObjectTracker(new_object_distance_threshold);
	// Publishers
	object_map_publisher = this->create_publisher<auv_msgs::msg::VisionObjectArray>(object_map_topic, 10);
	pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(vio_pose_topic, 10);
	// Subscriber for depth
	depth_subscriber = this->create_subscription<std_msgs::msg::Float64 >(
		"/sensors/depth", 10, std::bind(&ObjectMapNode::depth_callback, this, std::placeholders::_1)
	);
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
	void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
	{
#ifdef HAS_ZED_SDK
		if(!zed_detector) return;
		std::vector<sl::CustomBoxObjectData> zed_detections = extract_ZED_detections(msg);
		// Convert ROS message to ZED SDK CustomBoxObjectData
		zed_detector->process_detections(zed_detections);
		const auto [measurements,covariances,classes,orientations,confidences] = zed_detector->GetDetections();
		
		// The classes returned by `GetDetections` are strings derived from int label inside `ZEDDetection`.
		// So passing the correct int label is crucial.
		
		std::vector<Track> confirmed_tracks = object_tracker.update(
			measurements, 
			covariances, 
			classes, 
			orientations, 
			confidences
		);
		publish_pose(zed_detector->GetCameraPose());
		publish_object_map(confirmed_tracks);
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
		// TODO: implement publishing logic, use object_map_publisher
		auv_msgs::msg::VisionObjectArray object_map_msg;
		// for each track, publish as VisionObject
		for (const auto& track : tracks)
		{
			auv_msgs::msg::VisionObject object_msg;
			object_msg.label = track.label;
			object_msg.id = track.id;
			Eigen::Vector3d position = track.get_position();
			object_msg.x = position(0);
			object_msg.y = position(1);
			object_msg.z = position(2);
			object_msg.theta_z = 0.0; // TODO: set orientation if available
			object_msg.confidence = track.confidence;
			object_map_msg.array.push_back(object_msg);
		}
		object_map_publisher->publish(object_map_msg);
		// RCLCPP_DEBUG(this->get_logger(), "Published object map with %zu objects", tracks.size());
		RCLCPP_DEBUG(this->get_logger(), "Published object map with %zu objects", tracks.size());
		rclcpp::Time pipeline_end_time = this->now();
		rclcpp::Duration time_diff = pipeline_end_time - frame_collection_time;
		RCLCPP_DEBUG(this->get_logger(), "Object map pipeline latency: %.9f seconds", time_diff.seconds());
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
			
			// 2D Bounding Box
			// vision_msgs/BoundingBox2D is centered (cx, cy, w, h)
			// ZED SDK expects corners: top-left ?? No, let's check CustomBoxObjectData
			// "bounding_box_2d: 2D bounding box of the object in the image (4 corners)"
			
			float cx = detection.bbox.center.position.x;
			float cy = detection.bbox.center.position.y;
			float w = detection.bbox.size_x;
			float h = detection.bbox.size_y;
			
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
			
			// Label & Confidence (taking top hypothesis)
			if (!detection.results.empty()) {
				box.probability = detection.results[0].hypothesis.score;
                
                std::string label = detection.results[0].hypothesis.class_id;
                auto it = std::find(ID_TO_LABEL.begin(), ID_TO_LABEL.end(), label);
                if (it != ID_TO_LABEL.end()) {
                    box.label = std::distance(ID_TO_LABEL.begin(), it);
                } else {
                    // Fallback or skip?
                    // For now, let's keep it as is or default to something?
                    // If ZED receives an invalid label ID, it might crash or ignore.
                    // Let's assume 0 ("gate") or continue?
                    // Ideally we should skip this detection if label is unknown.
                    // But maybe we can just set a safe default.
                    box.label = -1; // Unknown
                    // But we should probably skip adding it to zed_detections if it's invalid.
                }
			}
			
            if (box.label != -1) {
			    box.is_grounded = false;
			    zed_detections.push_back(box);
            }
		}

		frame_collection_time = this->now();
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
