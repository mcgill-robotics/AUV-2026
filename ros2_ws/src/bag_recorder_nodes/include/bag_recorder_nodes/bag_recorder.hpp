#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/writer.hpp>
#include "bag_recorder_nodes/bag_recorder.hpp"

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder();
  
private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
