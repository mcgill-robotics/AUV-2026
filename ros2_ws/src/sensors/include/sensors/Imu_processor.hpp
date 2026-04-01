#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sensors/utils.hpp"
#include <string>



namespace sensors
{
using Vec3  = Eigen::Matrix<double,3,1>;
using Vec4  = Eigen::Matrix<double,4,1>;
using quatd = Eigen::Quaternion<double>;
using Mat3  = Eigen::Matrix<double,3,3>;
using imu_msg = sensor_msgs::msg::Imu;

struct ImuDataRawFrame {
    Vec3 f_s;    // Specific Force in the IMU's frame
    Vec3 w_s;    // Angular velocity in the IMU's frame
    quatd q_is;  // Sensor's orientation in the world frame
};

struct ImuDataAUVFrame {
    Vec3 a_free_v; // Free acceleration in AUV frame
    Vec3 w_v;    // Angular velocity in AUV frame
    quatd q_iv;  // Vehicle orientation relative to the world frame
};

class ImuProcessor: public rclcpp::Node
{
	public:
		ImuProcessor(); 
		~ImuProcessor() = default;

	private:
		void imu_callback(const imu_msg::SharedPtr imu_in);

		rclcpp::Publisher<imu_msg>::SharedPtr imu_pub_;
		rclcpp::Subscription<imu_msg>::SharedPtr imu_sub_;

		ImuDataRawFrame parse_imu(const imu_msg& imu_in) const;
		ImuDataAUVFrame process_imu(const ImuDataRawFrame& imu_raw) const;
		imu_msg compose_imu_msg(const ImuDataAUVFrame& imu_auv) const;

		Vec3 compute_free_acc(const Vec3& specific_force, const quatd& q_si) const;
		Vec3 rotate_gyro(const Vec3& w_s) const;

		quatd q_sv_;
		quatd q_vs_; 
		quatd q_in_;
		Vec3 g_i;
		quatd q_iv_;
		std::string frame_id_auv_;
		std::string frame_id_global_;			

};
} // namespace sensors
