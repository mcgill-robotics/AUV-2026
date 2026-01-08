#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace sensors
{
using Vec3  = Eigen::Matrix<double,3,1>;
using Vec4  = Eigen::Matrix<double,4,1>;
using quatd = Eigen::Quaternion<double>;
using Mat3  = Eigen::Matrix<double,3,3>;
using imu_msg = sensor_msgs::msg::Imu;

class ImuRepublisher
{
	public:
		explicit ImuRepublisher(const quatd& q_sv, const quatd& q_in); // consult README for variable naming convention. 
		imu_msg process(const imu_msg& imu_in);
		quatd q_vi_; 

	private:
		Vec3 compute_free_acc(const Vec3& specific_force, const quatd& q_si) const;
		Vec3 rotate_gyro(const Vec3& w_s) const;
		quatd q_sv_;
		quatd q_vs_; 
		quatd q_in_;
		Vec3 g_i;		

};
} // namespace sensors
