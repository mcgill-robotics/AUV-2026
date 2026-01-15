#include <chrono>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "auv_msgs/msg/vision_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <opencv2/opencv.hpp>
// #include <opencv2/dnn.hpp>

using namespace std;
// alias for milliseconds with double precision
using double_ms = std::chrono::duration<double, std::milli>;

enum ObjectClass
{
    GATE, LANE_MARKER, RED_PIPE, WHITE_PIPE, OCTAGON, TABLE, BIN, BOARD, SHARK, SAWFISH
};

struct DetectedObject
{
    ObjectClass class_id;
    float confidence;
    cv::Rect bbox; // x, y, width, height
};

array<string, 10> ID_TO_LABEL = 
{
    "gate",
    "lane_marker",
    "red_pipe",
    "white_pipe",
    "octagon",
    "table",
    "bin",
    "board",
    "shark",
    "sawfish"
};

class ObjectMapNode : public rclcpp::Node
{
public:
    ObjectMapNode() : Node("object_map_node")
    {
	// adding "this" boilerplate for methods called from rclcpp Node base class
	// YOLO model path
	this->declare_parameter("model_path", "");
	// how often to process frames for object mapping, i.e. publishing rate
	this->declare_parameter("frame_rate", 30);
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
	float new_object_distance_threshold;
	this->get_parameter("model_path", yolo_model_path);
	this->get_parameter("frame_rate", frame_rate);
	this->get_parameter("confidence_threshold", confidence_threshold);
	this->get_parameter("new_object_min_distance_threshold", new_object_distance_threshold);
	this->get_parameter("max_range", max_range_distance_threshold);
	this->get_parameter("use_stream", use_stream);
	this->get_parameter("stream_ip", stream_ip);
	this->get_parameter("stream_port", stream_port);
	this->get_parameter("show_detections", show_detections);
	this->get_parameter("debug_logs", debug_logs);

	// Publishers
	object_map_publisher_ = this->create_publisher<auv_msgs::msg::VisionObject>("/vision/object_map", 10);
	pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vision/vio_pose", 10);

	// YOLO & Zed

	RCLCPP_INFO(this->get_logger(), "Loading YOLO mode from %s.", yolo_model_path.c_str());
	

	this->timer_ = this->create_wall_timer(
		double_ms(1000.0 / frame_rate), std::bind(&ObjectMapNode::process_frame, this)
	);
	RCLCPP_INFO(this->get_logger(), "Object Map Node has been initialized with frame rate %d.", frame_rate);
    }
private:
	void process_frame()
	{
		RCLCPP_INFO(this->get_logger(), "Processing frame for object mapping...");
	}
	rclcpp::TimerBase::SharedPtr timer_;
	int frame_rate;
	float confidence_threshold;
	float max_range_distance_threshold;
	bool use_stream;
	string stream_ip;
	int stream_port;
	bool show_detections;
	bool debug_logs;

	rclcpp::Publisher<auv_msgs::msg::VisionObject>::SharedPtr object_map_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectMapNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}