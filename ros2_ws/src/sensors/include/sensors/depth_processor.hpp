#pragma once

#include <rclcpp/rclcpp.hpp>
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

class DepthProcessor: public rclcpp::Node
{
	public:
		 DepthProcessor(); // consult README for variable naming convention. 
		~DepthProcessor() = default;
	private:

		void depth_callback(const std_msgs::msg::Float64::SharedPtr depth_in) const;
		void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_in);

		// depth calibration
		void set_depth_calibration(double depth_min_actual, double depth_min_sensor, double depth_max_actual, double depth_max_sensor);

		double get_calibrated_depth(double base_depth) const;

		Vec3 r_vs_v_; // Vector from sensor frame to vehicle frame, expressed in vehicle frame	

		// depth calibration parameters
		bool calibrate_depth_;
		double depth_slope_;
		double depth_offset_;

                rclcpp::Publisher<float64_msg>::SharedPtr depth_processed_pub_;
                rclcpp::Subscription<float64_msg>::SharedPtr depth_sub_;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
		quatd q_iv_; // Current vehicle orientation 

};
} // namespace sensors
