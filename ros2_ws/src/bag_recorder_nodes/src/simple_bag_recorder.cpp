#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rosbag2_cpp/writer.hpp>
#include "bag_recorder_nodes/bag_recorder.hpp"

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp_lifecycle::LifecycleNode
{
public:
  SimpleBagRecorder() : LifeCycleNode("simple_bag_recorder")
  {
    RCLCPP_INFO(get_logger(), "SimpleBagRecorder node has been created");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    // Initialize the rosbag2 writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");

    // Create parameters for recording rates and topics to record
    this->declare_parameter<std::vector<std::string>>("topics_to_record", {"chatter"})
    this->declare_parameter<double>("recording_hz_pose", 10.0);
    this->declare_parameter<double>("recording_hz_imu", 10.0);
    this->declare_parameter<double>("recording_hz_images", 1.0);

    double param_hz_pose = this->get_parameter("topics_to_record", topics_to_record_);
    double param_hz_imu = this->get_parameter("topics_to_record", topics_to_record_);
    double param_hz_images = this->get_parameter("topics_to_record", topics_to_record_);

    // Create a subscription to the topics that are recordable
    subscription_pose = create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
    
    subscription_imu = create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
      
    subscription_images = create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
    RCLCPP_INFO(get_logger(), "SimpleBagRecorder node has been configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) 
  {
    subscription_->on_activate();

    RCLCPP_INFO(get_logger(), "SimpleBagRecorder node has been activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    subscription_->on_deactivate();

    RCLCPP_INFO(get_logger(), "SimpleBagRecorder node has been deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    subscription_.reset();
    writer_.reset();

    RCLCPP_INFO(get_logger(), "SimpleBagRecorder node has been cleaned up");
    return CallbackReturn::SUCCESS;
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    // Use the current ros time as the timestamp for the message being recorded
    // this is the way it is done by rosbag2 when recording messages
    rclcpp::Time time_stamp = this->now();

    writer_->write(msg, "chatter", "std_msgs/msg/String", time_stamp);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}