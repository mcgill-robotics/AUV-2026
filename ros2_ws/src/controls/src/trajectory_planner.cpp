#include "controls/trajectory_planner.hpp"
#include "sensors/utils.hpp"

namespace controls
{
        TrajectoryPlanner::TrajectoryPlanner(): Node("trajectory_planner")
        {
                sub_flip_command_ = this->create_subscription<auv_msgs::msg::FlipCommand>(
                        "/controls/flip_command",
                        1,
                        std::bind(&TrajectoryPlanner::flip_command_callback, this, std::placeholders::_1)
                );
                sub_reference_attitude_ = this->create_subscription<auv_msgs::msg::AttitudeReference>(
                        "/controls/attitude_reference",
                        1,
                        std::bind(&TrajectoryPlanner::reference_attitude_callback, this, std::placeholders::_1)
                );

                pub_reference_attitude_ = this->create_publisher<auv_msgs::msg::AttitudeReference>(
                        "/controls/attitude_reference",
                        1
                );

                trajectory_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<int64_t>(1000 / planner_rate_hz_)),
                        std::bind(&TrajectoryPlanner::trajectory_timer_callback, this)
                );
        }

        void TrajectoryPlanner::reference_attitude_callback(const auv_msgs::msg::AttitudeReference::SharedPtr msg)
        {
                if (!flip_in_progress_)
                {
                        // Update initial orientation for flip if not currently flipping
                        q_start_ = quatd(
                                msg->orientation.w,
                                msg->orientation.x,
                                msg->orientation.y,
                                msg->orientation.z
                        );
                }
        }

        void TrajectoryPlanner::flip_command_callback(const auv_msgs::msg::FlipCommand::SharedPtr msg)
        {
                flip_in_progress_ = true;

                if (msg->axis == auv_msgs::msg::FlipCommand::AXIS_X)
                {
                flip_axis_ = Vec3::UnitX();
                }
                else if (msg->axis == auv_msgs::msg::FlipCommand::AXIS_Y)
                {
                flip_axis_ = Vec3::UnitY();
                }
                else if (msg->axis == auv_msgs::msg::FlipCommand::AXIS_Z)
                {
                flip_axis_ = Vec3::UnitZ();
                }
                else
                {
                RCLCPP_WARN(this->get_logger(), "Invalid flip axis.");
                return;
}

                direction_ = msg->direction;
                count_ = msg->count;

                flip_duration_ = msg->flip_duration;
                total_duration_= flip_duration_ * count_;
                total_angle_rad_ = count_ * 2 * M_PI * direction_;

                flip_start_time_ = this->now();
        }

        void TrajectoryPlanner::trajectory_timer_callback()
        {
                if (flip_in_progress_)
                {
                        rclcpp::Time now = this->now();
                        double elapsed_sec = (now - flip_start_time_).seconds();

                        if (elapsed_sec >= total_duration_)
                        {
                                // Flip complete
                                flip_in_progress_ = false;
                                return;
                        }

                        float u = elapsed_sec / total_duration_;
                        float s= minimum_jerk(u);
                        float theta = s * total_angle_rad_;

                        quatd q_v1v2 = quatd::Identity();

                        q_v1v2.w() = std::cos(theta / 2.0);
                        q_v1v2.vec() = flip_axis_ * std::sin(theta / 2.0);

                        quatd q_target = q_start_ * q_v1v2;

                        float s_dot = minimum_jerk_derivative(u);
                        Vec3 w_ref_v = s_dot * flip_axis_ * total_angle_rad_/ total_duration_;

                        publish_attitude_reference(q_target, w_ref_v);
                }
        }

        void TrajectoryPlanner::publish_attitude_reference(const quatd& q, const Vec3& w_ref_v) const
        {
                auv_msgs::msg::AttitudeReference msg;
                msg.orientation.w = q.w();
                msg.orientation.x = q.x();
                msg.orientation.y = q.y();
                msg.orientation.z = q.z();
                msg.angular_velocity.x = w_ref_v.x();
                msg.angular_velocity.y = w_ref_v.y();
                msg.angular_velocity.z = w_ref_v.z();
                pub_reference_attitude_->publish(msg);
        }

        float TrajectoryPlanner::minimum_jerk(const float u) const
        {

                return 10 * std::pow(u, 3) - 15 * std::pow(u, 4) + 6 * std::pow(u, 5);
        }

        float TrajectoryPlanner::minimum_jerk_derivative(const float u) const
        {
                return 30 * std::pow(u, 2) - 60 * std::pow(u, 3) + 30 * std::pow(u, 4);
        }
}

int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);
        auto TrajectoryPlannerNode = std::make_shared<controls::TrajectoryPlanner>();
        rclcpp::spin(TrajectoryPlannerNode);
        rclcpp::shutdown();
        return 0;
}