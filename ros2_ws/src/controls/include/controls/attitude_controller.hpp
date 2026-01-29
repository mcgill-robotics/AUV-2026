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

    class AttitudeController: public rclcpp::Node
    {
        public:
                AttitudeController();
                ~AttitudeController() = default;

                wrench_msg compute_control_effort();                
                rclcpp::Publisher<wrench_msg>::SharedPtr pub_effort_; 
                
        private:
                // PID gains
                double P_ex_;
                double P_ey_;
                double P_ez_;
                double P_wx_;
                double P_wy_;
                double P_wz_;

                Mat3 P_e_;
                Mat3 P_w_;

                // Control Loop Frequency
                double control_loop_hz_;

                // AUV properties
                double buoyancy_;
                std::vector<double> r_bv_; // Position vector from body frame to vehicle center of buoyancy


                rclcpp::Subscription<imu_msg>::SharedPtr sub_imu_;
                rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_target_orientation_;

                // State variables
                quatd q_iv_;
                Vec3 w_iv_;

                //Target state variables
                quatd q_iv2_; 



                rclcpp::TimerBase::SharedPtr control_timer_;

                void imu_callback(const imu_msg::SharedPtr msg);
                void target_orientation_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg);
                Vec3 feedback_effort(const quatd& q_iv2);
                Vec3 feedforward_effort();
                void control_loop_callback();
                
    };

}