#include <chrono>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "auv_msgs/msg/vision_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#ifdef HAS_ZED_SDK
#include "object_map/zed_detection.hpp"
#endif
#include "object_map/object_tracker.hpp"

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
	int frame_rate;
	this->get_parameter("frame_rate", frame_rate);

	bool zed_sdk;
	this->get_parameter("zed_sdk", zed_sdk);
	bool sdk_available = false;
// compile time check for ZED SDK see CMakeLists.txt
#ifdef HAS_ZED_SDK
		sdk_available = true;
#else
		sdk_available = false;
#endif
	// final decision on using ZED SDK
	zed_sdk &&= sdk_available;
	if (!zed_sdk)
	{	
		this->declare_parameter<string>("front_cam_detection_topic", "/vision/front_cam/object_detection");
		string front_cam_detection_topic;
		this->get_parameter("front_cam_detection_topic", front_cam_detection_topic);
		RCLCPP_INFO(this->get_logger(), "ZED SDK not found, using front camera detection topic: %s", front_cam_detection_topic.c_str());
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "ZED SDK found, using ZED camera for object mapping.");
		// YOLO model path
		this->declare_parameter("model_path", "");
		// YOLO size set during training
		this->declare_parameter("yolo_input_size", 640);
		// probability threshold to consider a detection valid
		this->declare_parameter("confidence_threshold", 0.5);
		// maximum depth range to consider detections
		this->declare_parameter("max_range", 10.0);
		// minimum distance to consider a detection as a new object
		this->declare_parameter("new_object_min_distance_threshold", 0.5);
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
		float new_object_distance_threshold;
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
		this->get_parameter("new_object_min_distance_threshold", new_object_distance_threshold);
		this->get_parameter("max_range", max_range_distance_threshold);
		this->get_parameter("use_stream", use_stream);
		this->get_parameter("stream_ip", stream_ip);
		this->get_parameter("stream_port", stream_port);
		this->get_parameter("show_detections", show_detections);
		this->get_parameter("debug_logs", debug_logs);
	
		// Reinitialize zed_detector with actual parameters
		zed_detector.~ZEDDetection();
		new (&zed_detector) ZEDDetection(
			yolo_model_path,
			yolo_input_size,
			frame_rate,
			confidence_threshold,
			max_range_distance_threshold,
			new_object_distance_threshold,
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
	}

	// Publishers
	object_map_publisher = this->create_publisher<auv_msgs::msg::VisionObject>("/vision/object_map", 10);
	pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vision/vio_pose", 10);
	// Subscriber for depth
	depth_subscriber = this->create_subscription<std_msgs::msg::Float64 >(
		"/sensors/depth", 10, std::bind(&ObjectMapNode::depth_callback, this, std::placeholders::_1)
	);


	this->timer = this->create_wall_timer(
		double_ms(1000.0 / frame_rate), std::bind(&ObjectMapNode::frame_callback, this)
	);
	RCLCPP_INFO(this->get_logger(), "Object Map Node has been initialized with frame rate %d.", frame_rate);
    }
private:
	void frame_callback()
	{
#ifdef HAS_ZED_SDK
		zed_detector.process_frame();
#endif
	}

	void depth_callback(const std_msgs::msg::Float64::SharedPtr msg)
	{
#ifdef HAS_ZED_SDK
		zed_detector.UpdateSensorDepth(msg->data);
#endif
	}

#ifdef HAS_ZED_SDK
	ZEDDetection zed_detector;
#endif
	rclcpp::TimerBase::SharedPtr timer;

	rclcpp::Publisher<auv_msgs::msg::VisionObject>::SharedPtr object_map_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_subscriber;

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectMapNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}