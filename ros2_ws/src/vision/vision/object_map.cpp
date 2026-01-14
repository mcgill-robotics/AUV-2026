#include <chrono>

#include "rclcpp/rclcpp.hpp"

class ObjectMapNode : public rclcpp::Node
{
public:
    ObjectMapNode() : Node("object_map_node")
    {
		// alias for milliseconds with double precision
		using double_ms = std::chrono::duration<double, std::milli>;	
        this->declare_parameter("frame_rate", 30);
		this->get_parameter("frame_rate", frame_rate);
		double_ms frame_duration = double_ms(1000.0 / frame_rate);
		this->timer_ = this->create_wall_timer(
			frame_duration, std::bind(&ObjectMapNode::process_frame, this)
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
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectMapNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}