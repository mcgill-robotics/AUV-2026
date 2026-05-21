#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <message_filters/subscriber.h>
#include <auv_msgs/msg/attitude_reference.hpp>

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
                double P_ex_large_;
                double P_ey_large_;
                double P_ez_large_;
                double P_wx_large_;
                double P_wy_large_;
                double P_wz_large_;

                double P_ex_sas_;
                double P_ey_sas_;
                double P_ez_sas_;
                double P_wx_sas_;
                double P_wy_sas_;
                double P_wz_sas_;

                double sas_switch_deg_;
                double sas_switch_;

                Mat3 P_e_large_;
                Mat3 P_w_large_;

                Mat3 P_e_sas_;
                Mat3 P_w_sas_;

                enum class ControlMode
                {
                    LARGE_ERROR,
                    SAS
                };

                // Control Loop Frequency
                double control_loop_hz_;

                // AUV properties
                double buoyancy_;
                std::vector<double> r_bv_v_; // Position vector from body frame to center of buoyancy, expressed in body frame. 


                rclcpp::Subscription<imu_msg>::SharedPtr sub_imu_;
                rclcpp::Subscription<auv_msgs::msg::AttitudeReference>::SharedPtr sub_attitude_reference_;

                // State variables
                quatd q_iv_;
                Vec3 w_iv_v; // Vehicle angular velocity vector in the inertial frame, expressed in the vehicle frame

                //Target state variables
                quatd q_iv2_; 
                Vec3 w_ref_v; // Target (reference) angular velocity vector in the inertial frame, expressed in the body frame




                rclcpp::TimerBase::SharedPtr control_timer_;
                rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
                bool enabled_;

                void imu_callback(const imu_msg::SharedPtr msg);
                void target_attitude_callback(const auv_msgs::msg::AttitudeReference::SharedPtr msg);
                Vec3 feedback_effort(const quatd& q_iv2, const ControlMode& mode);
                Vec3 feedforward_effort();
                void control_loop_callback();
                rcl_interfaces::msg::SetParametersResult parameters_callback(
                    const std::vector<rclcpp::Parameter> &parameters
                );
                
    };

}
