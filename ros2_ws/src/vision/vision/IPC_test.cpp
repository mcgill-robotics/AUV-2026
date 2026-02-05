#include <memory>
#include <chrono>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

struct ZED_IPC_Consumer : public rclcpp::Node
{
public:
  ZED_IPC_Consumer(const std::string & name)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    std::string input_topic;
    this->declare_parameter<std::string>("input_topic", "/zed2/left/image_rect_color");
    this->get_parameter("input_topic", input_topic);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      1, // only take the latest image
      std::bind(&ZED_IPC_Consumer::process_message, this, _1)
    );
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  void process_message(const sensor_msgs::msg::Image::SharedPtr msg)
  {
      // Log the receipt of the message along with its memory address
      RCLCPP_INFO(this->get_logger(), "Received image of size: %dx%d with memory address: 0x%" PRIXPTR, msg->width, msg->height, reinterpret_cast<std::uintptr_t>(msg.get()));
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZED_IPC_Consumer>("zed_ipc_consumer"));
  rclcpp::shutdown();
  return 0;
}