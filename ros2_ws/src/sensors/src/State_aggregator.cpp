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
        this->declare_parameter<bool>("publish_pose_tf", true);
        this->declare_parameter<bool>("use_vio_for_position", false);
        this->declare_parameter<bool>("use_vio_for_orientation", false);
        this->get_parameter("frame_id_auv", frame_id_auv_);
        this->get_parameter("frame_id_global", frame_id_global_);
        this->get_parameter("publish_pose_tf", publish_pose_tf_);
        this->get_parameter("use_vio_for_position", use_vio_for_position_);
        this->get_parameter("use_vio_for_orientation", use_vio_for_orientation_);

        imu_sub_ = this->create_subscription<imu_msg>(
            "auv_frame/imu",
            rclcpp::SensorDataQoS().keep_last(1), // Use queue size 1 to get latest data
            std::bind(&State_aggregator::imu_callback, this, std::placeholders::_1)
        );
        depth_sub_ = this->create_subscription<float64_msg>(
            "auv_frame/depth",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&State_aggregator::depth_callback, this, std::placeholders::_1)
        );

        dvl_position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "auv_frame/dvl/position",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&State_aggregator::dvl_position_callback, this, std::placeholders::_1)
        );

        dvl_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "auv_frame/dvl/velocity",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&State_aggregator::dvl_velocity_callback, this, std::placeholders::_1)
        );

        vio_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "front_cam/vio/pose",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&State_aggregator::vio_pose_callback, this, std::placeholders::_1)
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "state/pose",
            rclcpp::SensorDataQoS().keep_last(1)
        );
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize state variables
        current_depth_ = 0.0;
        current_position_dvl_ = Vec3::Zero();
        current_velocity_ = Vec3::Zero();
        current_orientation_imu_.w = 1.0;
        current_orientation_imu_.x = 0.0;
        current_orientation_imu_.y = 0.0;
        current_orientation_imu_.z = 0.0;   
        current_position_vio_ = Vec3::Zero();
        current_orientation_vio_.w = 1.0;
        current_orientation_vio_.x = 0.0;
        current_orientation_vio_.y = 0.0;
        current_orientation_vio_.z = 0.0;   
        
        // Timer to publish aggregated state
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000 / publish_frequency_)),   // Control loop frequency
            std::bind(&State_aggregator::publish_state, this)
            );

    }

    // IMU callback
    void State_aggregator::imu_callback(const imu_msg::SharedPtr imu_in)
    {
        current_orientation_imu_ = imu_in->orientation;
    }

    // Depth callback
    void State_aggregator::depth_callback(const float64_msg::SharedPtr depth_in)
    {
        current_depth_ = depth_in->data;
    }

    // DVL position callback
    void State_aggregator::dvl_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr position_in)
    {
        current_position_dvl_(0) = position_in->point.x;
        current_position_dvl_(1) = position_in->point.y;
        current_position_dvl_(2) = position_in->point.z;
    }

    // DVL velocity callback
    void State_aggregator::dvl_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr velocity_in)
    {
        current_velocity_(0) = velocity_in->twist.linear.x;
        current_velocity_(1) = velocity_in->twist.linear.y;
        current_velocity_(2) = velocity_in->twist.linear.z;
    }

    // VIO pose callback
    void State_aggregator::vio_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_in)
    {
        current_position_vio_(0) = pose_in->pose.position.x;
        current_position_vio_(1) = pose_in->pose.position.y;
        current_position_vio_(2) = pose_in->pose.position.z;
        current_orientation_vio_ = pose_in->pose.orientation;
    }

    // Publish aggregated state
    void State_aggregator::publish_state()
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = frame_id_global_;

        pose_msg.pose.orientation = use_vio_for_orientation_ ? current_orientation_vio_ : current_orientation_imu_;
        pose_msg.pose.position.x = use_vio_for_position_ ? current_position_vio_(0) : current_position_dvl_(0);
        pose_msg.pose.position.y = use_vio_for_position_ ? current_position_vio_(1) : current_position_dvl_(1);
        pose_msg.pose.position.z = -current_depth_; // Down is negative Z

        pose_pub_->publish(pose_msg);

        if (publish_pose_tf_) {
            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header = pose_msg.header;
            transform_msg.child_frame_id = frame_id_auv_;
            transform_msg.transform.translation.x = pose_msg.pose.position.x;
            transform_msg.transform.translation.y = pose_msg.pose.position.y;
            transform_msg.transform.translation.z = pose_msg.pose.position.z;
            transform_msg.transform.rotation = pose_msg.pose.orientation;
            tf_broadcaster_->sendTransform(transform_msg);
        }
    }
} // namespace sensors

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::State_aggregator>());
	rclcpp::shutdown();

	return 0;
}
	
