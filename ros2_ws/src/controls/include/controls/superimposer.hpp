#pragma once

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <message_filters/subscriber.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <functional>

namespace controls
{
    using float64msg = std_msgs::msg::Float64;
    using wrench_msg = geometry_msgs::msg::Wrench;
    using imu_msg = sensor_msgs::msg::Imu;
    using quatd = Eigen::Quaternion<double>;
    using Mat3 = Eigen::Matrix3d;
    using Vec3 = Eigen::Vector3d;

    class superimposer: public rclcpp::Node
        {
                public:
                        superimposer();
                        ~superimposer() = default;

                private:
                        // Subscribe to processed imu data to get orientation
                        rclcpp::Subscription<imu_msg>::SharedPtr sub_imu_;

                        // Subscribe to efforts from controllers
                        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_depth_effort_;
                        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_attitude_effort_;
                        // TODO: Add linear (X,Y) controllers when available
                        

                        //Publish combined effort for propulsion package
                        rclcpp::Publisher<wrench_msg>::SharedPtr pub_effort_;
                        rclcpp::TimerBase::SharedPtr publish_timer_;
                        
                        void imu_callback(const imu_msg::SharedPtr msg);
                        void depth_effort_callback(const wrench_msg::SharedPtr msg);
                        void attitude_effort_callback(const wrench_msg::SharedPtr msg);
                        void publish_combined_effort();
                        
                        
                        quatd q_vi_;
                        wrench_msg depth_effort_; // Last received depth effort
                        wrench_msg attitude_effort_; // Last received attitude effort
                        std::unique_ptr<double> effort_bias_force_x; // Bias effort to be added to the combined effort (optional)
                        std::unique_ptr<double> effort_bias_force_y; 
                        std::unique_ptr<double> effort_bias_force_z; 
                        std::unique_ptr<double> effort_bias_torque_x; 
                        std::unique_ptr<double> effort_bias_torque_y; 
                        std::unique_ptr<double> effort_bias_torque_z;
                        double publish_hz_;


        };









}