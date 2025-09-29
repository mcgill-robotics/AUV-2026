#ifndef SENSORS_MOVELLA_HPP
#define SENSORS_MOVELLA_HPP

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

namespace sensors
{

class MovellaRepublisher : public rclcpp::Node
{
	public:
		MovellaRepublisher();

	private:
		void callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg, 
			      const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &free_acc_msg);

		// Publishers
		rclcpp::Publisher<sensor_msgs::msg::Imu> pub_imu = std::make_shared<rclcpp::Publisher<sensor_msgs::msg::Imu>;

		// Message filter sub and synchronizer type
		message_filters::Subscriber<sensor_msgs::msg::Imu> sub_imu;
		message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> sub_free_acc_imu;

		using SyncPolicy = message_filters::sync_policies::ApproximateTime< \
				   sensor_msgs::msg::Imu, geometry_msgs::msg::Vector3Stamped>; //  //ApproximateTime is a message filter algorithm that matches messages with similar timestamps
		message_filters::Synchronizer<SyncPolicy> sync = std::make_shared<SyncPolicy>;

		// Parameters
		std::string frame_override;
		int param_framerate_pub;
};

}

#endif

