#pragma once

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sensors
{
using Vec3  = Eigen::Matrix<double,3,1>;
using Vec4  = Eigen::Matrix<double,4,1>;
using quatd = Eigen::Quaternion<double>; // Eigen convention: [w, x, y, z], ROS convention: [x, y, z, w]
using Mat3  = Eigen::Matrix<double,3,3>;
using float64_msg = std_msgs::msg::Float64;

class DepthRepublisher
{
	public:
		explicit DepthRepublisher(const Vec3& r_vs); // consult README for variable naming convention. 
		float64_msg process(const float64_msg& depth_in, const quatd& q_vi) const;	
	private:
		Vec3 r_vs_v_; // Vector from sensor frame to vehicle frame, expressed in vehicle frame	

};
} // namespace sensors
