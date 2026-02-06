#include <memory>
#include <chrono>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
struct ZED_IPC_Consumer : public rclcpp::Node
{
public:
  explicit ZED_IPC_Consumer(const rclcpp::NodeOptions & options)
  : Node("zed_ipc_consumer", options)
  {
    this->declare_parameter<std::string>("input_topic", "/zed2/left/image_rect_color");
    this->get_parameter("input_topic", input_topic);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      1, // only take the latest image
      std::bind(&ZED_IPC_Consumer::process_message, this, _1)
    );
    //timer_ = this->create_wall_timer(500ms, std::bind(&ZED_IPC_Consumer::print_shit, this));
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string input_topic;
  void process_message(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Log the receipt of the message along with its memory address
    RCLCPP_INFO(this->get_logger(), "Received image of size: %dx%d with memory address: 0x%" PRIXPTR, msg->width, msg->height, reinterpret_cast<std::uintptr_t>(msg.get()));
  }
  void print_shit()
  {
    RCLCPP_INFO(this->get_logger(), "Input Topic:%s",input_topic.c_str());
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(ZED_IPC_Consumer);