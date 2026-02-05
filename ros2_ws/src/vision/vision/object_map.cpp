#include <chrono>
#include <string>
#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "auv_msgs/msg/vision_object.hpp"
#include "auv_msgs/msg/vision_object_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#ifdef HAS_ZED_SDK
	#include "zed_detection.hpp"
#endif
#include "object_tracker.hpp"

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
	// how often to process frames for object mapping, i.e. publishing rate
	this->declare_parameter("frame_rate", 30);
	// minimum distance to consider a detection as a new object
	this->declare_parameter("new_object_min_distance_threshold", 0.5);
	
	// only needed in constructor
	int frame_rate;
	float new_object_distance_threshold;
	this->get_parameter("frame_rate", frame_rate);
	this->get_parameter("new_object_min_distance_threshold", new_object_distance_threshold);
	this->get_parameter("zed_sdk", zed_sdk);
	bool sdk_available = false;
// compile time check for ZED SDK see CMakeLists.txt
#ifdef HAS_ZED_SDK
		sdk_available = true;
#endif
	// final decision on using ZED SDK
	zed_sdk &= sdk_available;
	if (!zed_sdk)
	{	
		this->declare_parameter<string>("front_cam_detection_topic", "/vision/front_cam/object_detection");
		string front_cam_detection_topic;
		this->get_parameter("front_cam_detection_topic", front_cam_detection_topic);
		RCLCPP_INFO(this->get_logger(), "ZED SDK not found, using front camera detection topic: %s", front_cam_detection_topic.c_str());
		front_cam_subscriber = this->create_subscription<auv_msgs::msg::VisionObjectArray>(
			front_cam_detection_topic, 10, std::bind(&ObjectMapNode::front_cam_callback, this, std::placeholders::_1)
		);
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "ZED SDK found, using ZED camera for object mapping.");
		// TODO: refactor these 5 into a YOLO/Detector config file
		// YOLO model path
		this->declare_parameter("model_path", "");
		// YOLO size set during training
		this->declare_parameter("yolo_input_size", 640);
		// probability threshold to consider a detection valid
		this->declare_parameter("YOLO_confidence_threshold", 0.5);
		// Same as YOLO confidence but Intersection over Union metric
		this->declare_parameter("YOLO_IOU_threshold", 0.1);
		// maximum depth range to consider detections
		this->declare_parameter("max_range", 10.0);
		// use UDP stream for input frames
		this->declare_parameter("use_stream", true);
		// UDP stream IP and port
		this->declare_parameter("stream_ip", "127.0.0.1");
		this->declare_parameter("stream_port", 30000);
		// show YOLO bounding boxes on output frames
		this->declare_parameter("show_detections", true);
		// enable heavy debug table logs
		this->declare_parameter("debug_logs", false);
	
		// get parameters
		// only needed in constructor
		string yolo_model_path;
		int yolo_input_size;
		int frame_rate;
		float confidence_threshold;
		float max_range_distance_threshold;
		bool use_stream;
		string stream_ip;
		int stream_port;
		bool show_detections;
		bool debug_logs;
		this->get_parameter("model_path", yolo_model_path);
		this->get_parameter("yolo_input_size", yolo_input_size);
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
			yolo_model_path,
			yolo_input_size,
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
	object_map_publisher = this->create_publisher<auv_msgs::msg::VisionObjectArray>("/vision/object_map", 10);
	pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vision/vio_pose", 10);
	// Subscriber for depth
	depth_subscriber = this->create_subscription<std_msgs::msg::Float64 >(
		"/sensors/depth", 10, std::bind(&ObjectMapNode::depth_callback, this, std::placeholders::_1)
	);
// ZED SDK requires querying, whereas for ROS topics we use callbacks 
#ifdef HAS_ZED_SDK
	if(zed_sdk)
	{
		this->timer = this->create_wall_timer(
		double_ms(1000.0 / frame_rate), std::bind(&ObjectMapNode::frame_callback, this)
		);
	}
#endif
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
	void frame_callback()
	{
#ifdef HAS_ZED_SDK
		// TODO: add time checks, make sure time has actually passed before process frame, may lead to div by 0
		frame_collection_time = this->now();
		zed_detector->process_frame();
		const auto [measurements,covariances,classes,orientations,confidences] = zed_detector->GetDetections();
		std::vector<Track> confirmed_tracks = object_tracker.update(
			measurements, 
			covariances, 
			classes, 
			orientations, 
			confidences
		);
		publish_object_map(confirmed_tracks);
		publish_pose(zed_detector->GetCameraPose());
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
	std::unique_ptr<ZEDDetection> zed_detector;
#endif

	ObjectTracker object_tracker;
	rclcpp::TimerBase::SharedPtr timer;

	bool zed_sdk;
	rclcpp::Time frame_collection_time;
	rclcpp::Publisher<auv_msgs::msg::VisionObjectArray>::SharedPtr object_map_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_subscriber;
	// if zed SDK not in use
	rclcpp::Subscription<auv_msgs::msg::VisionObjectArray>::SharedPtr front_cam_subscriber;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectMapNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
