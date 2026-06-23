#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <auv_msgs/msg/flip_command.hpp>      
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <message_filters/subscriber.h>
#include <auv_msgs/msg/attitude_reference.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <functional>

namespace controls
{
    using quatd = Eigen::Quaternion<double>;
        class TrajectoryPlanner : public rclcpp::Node
{
        public:
                TrajectoryPlanner();
                ~TrajectoryPlanner() = default;

        private:
                using Vec3 = Eigen::Vector3d;

                rclcpp::Subscription<auv_msgs::msg::FlipCommand>::SharedPtr sub_flip_command_;
                rclcpp::Subscription<auv_msgs::msg::AttitudeReference>::SharedPtr sub_reference_attitude_;
                rclcpp::Publisher<auv_msgs::msg::AttitudeReference>::SharedPtr pub_reference_attitude_;

                rclcpp::TimerBase::SharedPtr trajectory_timer_;

                void flip_command_callback(const auv_msgs::msg::FlipCommand::SharedPtr msg);
                void reference_attitude_callback(const auv_msgs::msg::AttitudeReference::SharedPtr msg);
                void trajectory_timer_callback();

                void publish_attitude_reference(const quatd& q, const Vec3& w_ref_v) const;
                float minimum_jerk(float u) const;
                float minimum_jerk_derivative(float u) const;

                quatd q_start_ = quatd::Identity();

                bool flip_in_progress_ = false;

                float trajectory_planner_rate_hz_ = 10.0;

                rclcpp::Time flip_start_time_;

                Vec3 flip_axis_ = Vec3::UnitX();

                int direction_;
                uint32_t count_;

                float flip_duration_;
                float total_duration_ ;
                float total_angle_rad_ ;
                };
}      