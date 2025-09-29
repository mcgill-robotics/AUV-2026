#include <rclcpp/rclcpp.hpp>
#include <sensors/imu_movella.hpp> 

class MovellaRepublisher : public rclcpp::Node
{
	public:
		MovellaRepublisher() : Node("movella_republisher")
	{
		// Create subscribers and publisher
		pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("sensor/imu/data", param_framerate_pub);
		sub_imu.subscribe(this, "imu/data"); //uses .subscribe instead of create_subscription because using messageFilters
		sub_free_acc_imu.subscribe(this, "filter/free_acceleration");
	

		// Create a synchronizer that synchs the free_acc message and regular imu message 
		// with an interval of 0.05s allowed time and calls the callback func to process the messages
		sync.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), sub_imu, sub_free_acc_imu));
		sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
		sync->registerCallback(std::bind(&MovellaRepublisher::callback, this, std::placeholders::_1, std::placeholders::_2));

		// Set parameters
		this->declare_parameter<std::string>("frame_id", "imu"); //create default param value to frame_id
		this->get_parameter("frame_id", frame_override);         //override in case launch file has a desired value and same with pub rate
		this->declare_parameter<int>("framerate_pub", 20);
		this->get_parameter("framerate_pub", param_framerate_pub); 
	}
		
	private:
		void callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
				const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& free_acc_msg) const
		{

			// Takes imu_msg and imu without acceleration message and fuse their data into one message
			// for state estimation
			 
			
			// Prepare new header
			sensor_msgs::msg::Imu processed_imu_msg;

			processed_imu_msg.header.stamp = imu_msg->header.stamp;
			processed_imu_msg.header.frame_id = frame_override.empty() ? imu_msg->header.frame_id : frame_override;

			// Prepare new data content 
			processed_imu_msg.orientation  = imu_msg->orientation;
			processed_imu_msg.orientation_covariance = imu_msg->orientation_covariance;
			processed_imu_msg.angular_velocity = imu_msg->angular_velocity;
			processed_imu_msg.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
			processed_imu_msg.linear_acceleration = free_acc_msg->vector;
			processed_imu_msg.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;

			// Check if the two fused messages are too far apart time wise
			double dt = fabs((imu_msg->header.stamp.nanosec - free_acc_msg->header.stamp.nanosec) * 1000000);

			if (dt > 0.015) 
			{
				RCLCPP_WARN_THROTTLE(this->get_logger(), *const_cast<rclcpp::Clock*>(this->get_clock().get()), 5000, 
						"IMU vs FreeAccel timestamp diff: %.1f ms", dt * 1000.0);
			}

			// Publish new message
			pub_imu->publish(processed_imu_msg);
		}

};


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MovellaRepublisher>());
	rclcpp::shutdown();

	return 0;
}

			

			

