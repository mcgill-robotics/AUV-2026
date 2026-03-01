#include "sensors/depth_processor.hpp"
#include "sensors/Imu_processor.hpp"
#include "sensors/State_aggregator.hpp"

// Main Node that republish processed IMU and Depth data
namespace sensors
{
    State_aggregator::State_aggregator()
        : Node("state_aggregator")
    {
        this->declare_parameter<double>("publish_frequency", 50.0); // Hz
        this->get_parameter("publish_frequency", publish_frequency_);

        this->declare_parameter<std::string>("frame_id_auv", "auv_link");
        this->declare_parameter<std::string>("frame_id_global", "pool_link");
        this->get_parameter("frame_id_auv", frame_id_auv_);
        this->get_parameter("frame_id_global", frame_id_global_);



        imu_sub_ = this->create_subscription<imu_msg>(
            "auv_frame/imu",
            1, // Use queue size 1 to get latest data
            std::bind(&State_aggregator::imu_callback, this, std::placeholders::_1)
        );
        depth_sub_ = this->create_subscription<float64_msg>(
            "auv_frame/depth",
            1,
            std::bind(&State_aggregator::depth_callback, this, std::placeholders::_1)
        );

        dvl_position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "auv_frame/dvl/position",
            1,
            std::bind(&State_aggregator::dvl_position_callback, this, std::placeholders::_1)
        );

        dvl_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "auv_frame/dvl/velocity",
            1,
            std::bind(&State_aggregator::dvl_velocity_callback, this, std::placeholders::_1)
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "state/pose",
            1
        );

        // Initialize state variables
        current_depth_ = 0.0;
        current_position_dvl_ = Vec3::Zero();
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

    // DVL position callback
    void State_aggregator::dvl_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr position_in)
    {
        // Update current position from DVL
        current_position_dvl_(0) = position_in->point.x;
        current_position_dvl_(1) = position_in->point.y;
        current_position_dvl_(2) = position_in->point.z;
    }

    // DVL velocity callback
    void State_aggregator::dvl_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr velocity_in)
    {
        // Update current velocity from DVL
        current_velocity_(0) = velocity_in->twist.linear.x;
        current_velocity_(1) = velocity_in->twist.linear.y;
        current_velocity_(2) = velocity_in->twist.linear.z;
    }

    // Publish aggregated state
    void State_aggregator::publish_state()
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = frame_id_global_;

        pose_msg.pose.orientation = current_orientation_;
        pose_msg.pose.position.x = current_position_dvl_(0);
        pose_msg.pose.position.y = current_position_dvl_(1);
        pose_msg.pose.position.z = -current_depth_; // Down is negative Z

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
	