#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
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

class DepthProcessor: public rclcpp::Node
{
	public:
		 DepthProcessor(); // consult README for variable naming convention. 
		~DepthProcessor() = default;
	private:

		void depth_callback(const std_msgs::msg::Float64::SharedPtr depth_in);
		void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_in);
		void calibrate_callback(
			const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
			std::shared_ptr<std_srvs::srv::Trigger::Response> response);

		Vec3 r_vs_v_; // Vector from sensor frame to vehicle frame, expressed in vehicle frame	
                rclcpp::Publisher<float64_msg>::SharedPtr depth_pub_;
                rclcpp::Subscription<float64_msg>::SharedPtr depth_sub_;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
		quatd q_iv_; // Current vehicle orientation 

		// Calibration variables
		double zero_offset_;
		bool calibration_active_;
		int calibration_sample_count_;
		double calibration_sample_sum_;
		static constexpr int calibration_window_size_ = 5; 

};
} // namespace sensors
