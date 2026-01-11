#include "sensors/depth_processor.hpp"
#include "sensors/Imu_processor.hpp"
#include "sensors/State_aggregator.hpp"

// Main Node that republish processed IMU and Depth data
namespace sensors
{
    State_aggregator::State_aggregator()
        : Node("state_aggregator")
    {
        this->declare_parameter<std::vector<double>>("publish_frequency", {50.0}); // Hz
        this->get_parameter("publish_frequency", publish_frequency_);



        imu_sub_ = this->create_subscription<imu_msg>(
            "processed/imu",
            1, // Use queue size 1 to get latest data
            std::bind(&State_aggregator::imu_callback, this, std::placeholders::_1)
        );
        depth_sub_ = this->create_subscription<float64_msg>(
            "processed/depth",
            1,
            std::bind(&State_aggregator::depth_callback, this, std::placeholders::_1)
        );


        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "state/pose",
            1
        );

        // Initialize state variables
        current_depth_ = 0.0;
        current_position_ = Vec3::Zero();
        current_velocity_ = Vec3::Zero();
        current_orientation_.w = 1.0;
        current_orientation_.x = 0.0;
        current_orientation_.y = 0.0;
        current_orientation_.z = 0.0;   
        
        // Timer to publish aggregated state
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000 / publish_frequency_)),   // Control loop frequency
            std::bind(&State_aggregator::publish_state, this)
            );

    }

    // IMU callback
    void State_aggregator::imu_callback(const imu_msg::SharedPtr imu_in)
    {
        // Update current orientation from IMU
        current_orientation_ = imu_in->orientation;
    }

    // Depth callback
    void State_aggregator::depth_callback(const float64_msg::SharedPtr depth_in)
    {
        // Update current depth from Depth sensor
        current_depth_ = depth_in->data;
    }

    // Publish aggregated state
    void State_aggregator::publish_state()
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "auv";

        pose_msg.pose.orientation = current_orientation_;
        pose_msg.pose.position.x = current_position_(0);
        pose_msg.pose.position.y = current_position_(1);
        pose_msg.pose.position.z = -current_depth_; // Assuming down is negative z

        pose_pub_->publish(pose_msg);
}
} // namespace sensors

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::State_aggregator>());
	rclcpp::shutdown();

	return 0;
}
	