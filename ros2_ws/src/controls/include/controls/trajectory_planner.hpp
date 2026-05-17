#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <auv_msgs/msg/flip_command.hpp>      
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <message_filters/subscriber.h>

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
                rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_target_orientation_;
                rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_target_orientation_;

                rclcpp::TimerBase::SharedPtr trajectory_timer_;

                void flip_command_callback(const auv_msgs::msg::FlipCommand::SharedPtr msg);
                void target_orientation_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg);
                void trajectory_timer_callback();

                void publish_quaternion(const quatd& q) const;
                float minimum_jerk(float u) const;

                quatd q_start_ = quatd::Identity();

                bool flip_in_progress_ = false;

                float planner_rate_hz_ = 10.0;

                rclcpp::Time flip_start_time_;

                Vec3 flip_axis_ = Vec3::UnitX();

                int direction_;
                uint32_t count_;

                float flip_duration_;
                float total_duration_ ;
                float total_angle_rad_ ;
                };
}      