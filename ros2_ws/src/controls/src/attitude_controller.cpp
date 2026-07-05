#include "controls/attitude_controller.hpp"
#include "sensors/utils.hpp"

namespace controls
{
    AttitudeController::AttitudeController(): Node("attitude_controller")
    {
        this->declare_parameter<double>("P_ex", 1.0);
        this->declare_parameter<double>("P_ey", 1.0);
        this->declare_parameter<double>("P_ez", 1.0);
        this->declare_parameter<double>("I_ex", 0.0);
        this->declare_parameter<double>("I_ey", 0.0);
        this->declare_parameter<double>("I_ez", 0.0);
        this->declare_parameter<double>("P_wx", 1.0);
        this->declare_parameter<double>("P_wy", 1.0);
        this->declare_parameter<double>("P_wz", 1.0);
        this->declare_parameter<double>("I_MAX", 10.0);
        this->declare_parameter<double>("integral_activation_threshold_deg", std::numeric_limits<double>::infinity());
        this->declare_parameter<double>("max_slew_rate_roll_deg", 0.0);
        this->declare_parameter<double>("max_slew_rate_pitch_deg", 0.0);
        this->declare_parameter<double>("max_slew_rate_yaw_deg", 0.0);
        this->declare_parameter<double>("buoyancy", 278.0); // Newtons
        this->declare_parameter<std::vector<double>>("r_bv_v", {0.0, 0.0, 0.023}); // [m] From CAD Model
        this->declare_parameter<double>("control_loop_hz", 25.0); // Control loop frequency
        this->declare_parameter<bool>("enabled", false);

        this->get_parameter("P_ex", P_ex_);
        this->get_parameter("P_ey", P_ey_);
        this->get_parameter("P_ez", P_ez_);
        this->get_parameter("I_ex", I_ex_);
        this->get_parameter("I_ey", I_ey_);
        this->get_parameter("I_ez", I_ez_);
        this->get_parameter("P_wx", P_wx_);
        this->get_parameter("P_wy", P_wy_);
        this->get_parameter("P_wz", P_wz_);
        this->get_parameter("I_MAX", I_MAX_);
        this->get_parameter("integral_activation_threshold_deg", integral_activation_threshold_deg_);
        integral_activation_threshold_rad_ = integral_activation_threshold_deg_ * (M_PI / 180.0);
        this->get_parameter("max_slew_rate_roll_deg", max_slew_rate_roll_deg_);
        this->get_parameter("max_slew_rate_pitch_deg", max_slew_rate_pitch_deg_);
        this->get_parameter("max_slew_rate_yaw_deg", max_slew_rate_yaw_deg_);
        max_slew_rate_roll_rad_ = max_slew_rate_roll_deg_ * (M_PI / 180.0);
        max_slew_rate_pitch_rad_ = max_slew_rate_pitch_deg_ * (M_PI / 180.0);
        max_slew_rate_yaw_rad_ = max_slew_rate_yaw_deg_ * (M_PI / 180.0);
        this->get_parameter("buoyancy", buoyancy_);
        this->get_parameter("r_bv_v", r_bv_v_);
        this->get_parameter("control_loop_hz", control_loop_hz_);
        this->get_parameter("enabled", enabled_);
        was_enabled_ = enabled_;

        q_iv_ = quatd::Identity(); // Initial orientation: identity quaternion
        w_iv_ = Vec3::Zero(); // Initial angular velocity: zero vector
        q_iv2_ = quatd::Identity(); // Initial target orientation: identity quaternion
        target_q_iv2_ = quatd::Identity();

        integral_error_ = Vec3::Zero();

        P_e_ << P_ex_, 0, 0,
                0, P_ey_, 0,
                0, 0, P_ez_;
        
        I_e_ << I_ex_, 0, 0,
                0, I_ey_, 0,
                0, 0, I_ez_;
        
        P_w_ << P_wx_, 0, 0,
                0, P_wy_, 0,
                0, 0, P_wz_;


        pub_effort_ = this->create_publisher<wrench_msg>("/controls/attitude_effort", rclcpp::SensorDataQoS().keep_last(1));
        sub_imu_ = this->create_subscription<imu_msg>(
            "auv_frame/imu",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&AttitudeController::imu_callback, this, std::placeholders::_1)
        );
        sub_target_orientation_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/controls/quaternion_setpoint",
            1,
            std::bind(&AttitudeController::target_orientation_callback, this, std::placeholders::_1)
        );

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&AttitudeController::parameters_callback, this, std::placeholders::_1)
        );

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000 / control_loop_hz_)),   // Control loop frequency
            std::bind(&AttitudeController::control_loop_callback, this)
            );
    }

    void AttitudeController::imu_callback(const imu_msg::SharedPtr msg)
    {
        // Extract orientation quaternion from IMU message
        q_iv_ = quatd(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z
        );

        // Extract angular velocity vector from IMU message
        w_iv_ = Vec3(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
    }

    void AttitudeController::target_orientation_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
    {
        target_q_iv2_ = quatd(
            msg->w,
            msg->x,
            msg->y,
            msg->z
        );
        if ((max_slew_rate_roll_rad_ <= 0.0 && max_slew_rate_pitch_rad_ <= 0.0 && max_slew_rate_yaw_rad_ <= 0.0) || !enabled_) {
            q_iv2_ = target_q_iv2_;
        }
    }

    Vec3 AttitudeController::feedback_effort(const quatd& q_iv2)
    {
        quatd q_error = q_iv_.conjugate() * q_iv2;
        q_error = sensors::math::canonicalizeShortest(q_error);
        Vec3 error_vector = Vec3(q_error.x(), q_error.y(), q_error.z());

        double dt = 1.0 / control_loop_hz_;

        // Conditional integration based on magnitude of the error vector
        if (error_vector.norm() <= integral_activation_threshold_rad_) {
            integral_error_ += error_vector * dt;
            integral_error_ = integral_error_.cwiseMin(I_MAX_).cwiseMax(-I_MAX_);
        }

        Vec3 feedback = P_e_ * error_vector + I_e_ * integral_error_ - P_w_ * w_iv_;
        return feedback;
    }

    Vec3 AttitudeController::feedforward_effort()
    {
        // Compute the torque due to buoyancy offset
        Vec3 r_bv_vec(r_bv_v_[0], r_bv_v_[1], r_bv_v_[2]);
        Vec3 f_buoyancy = q_iv_.conjugate() * Vec3(0, 0, buoyancy_);
        Vec3 torque_buoyancy = r_bv_vec.cross(f_buoyancy);
        Vec3 feedforward = -1 * torque_buoyancy; // Negate to counteract
        return feedforward;
    }

    wrench_msg AttitudeController::compute_control_effort()
    {
        Vec3 feedback = feedback_effort(q_iv2_);
        Vec3 feedforward = feedforward_effort();
        Vec3 total_torque = feedback + feedforward;

        wrench_msg effort_msg;
        effort_msg.torque.x = total_torque.x();
        effort_msg.torque.y = total_torque.y();
        effort_msg.torque.z = total_torque.z();
        effort_msg.force.x = 0.0;
        effort_msg.force.y = 0.0;
        effort_msg.force.z = 0.0;

        return effort_msg;
    }

    void AttitudeController::control_loop_callback()
    {
        // Apply setpoint slew-rate limiting if enabled
        if (enabled_ && (max_slew_rate_roll_rad_ > 0.0 || max_slew_rate_pitch_rad_ > 0.0 || max_slew_rate_yaw_rad_ > 0.0)) {
            quatd q_diff = q_iv2_.conjugate() * target_q_iv2_;
            q_diff = sensors::math::canonicalizeShortest(q_diff);
            Eigen::AngleAxisd aa(q_diff);
            double angle = aa.angle();
            if (std::abs(angle) > 1e-6) {
                Vec3 d_angle = angle * aa.axis();
                double dt = 1.0 / control_loop_hz_;
                if (max_slew_rate_roll_rad_ > 0.0) {
                    double max_step = max_slew_rate_roll_rad_ * dt;
                    if (std::abs(d_angle.x()) > max_step) d_angle.x() = std::copysign(max_step, d_angle.x());
                }
                if (max_slew_rate_pitch_rad_ > 0.0) {
                    double max_step = max_slew_rate_pitch_rad_ * dt;
                    if (std::abs(d_angle.y()) > max_step) d_angle.y() = std::copysign(max_step, d_angle.y());
                }
                if (max_slew_rate_yaw_rad_ > 0.0) {
                    double max_step = max_slew_rate_yaw_rad_ * dt;
                    if (std::abs(d_angle.z()) > max_step) d_angle.z() = std::copysign(max_step, d_angle.z());
                }
                double new_angle = d_angle.norm();
                if (new_angle > 1e-6) {
                    quatd q_step(Eigen::AngleAxisd(new_angle, d_angle / new_angle));
                    q_iv2_ = (q_iv2_ * q_step).normalized();
                } else {
                    q_iv2_ = target_q_iv2_;
                }
            } else {
                q_iv2_ = target_q_iv2_;
            }
        } else {
            q_iv2_ = target_q_iv2_;
        }

        if (enabled_)
        {
            wrench_msg effort = compute_control_effort();
            pub_effort_->publish(effort);
            was_enabled_ = true;
        }
        else if (was_enabled_)
        {
            wrench_msg effort;
            effort.force.x = 0.0;
            effort.force.y = 0.0;
            effort.force.z = 0.0;
            effort.torque.x = 0.0;
            effort.torque.y = 0.0;
            effort.torque.z = 0.0;
            pub_effort_->publish(effort);
            was_enabled_ = false;
        }
    }

    rcl_interfaces::msg::SetParametersResult AttitudeController::parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    )
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "enabled")
            {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    result.successful = false;
                    result.reason = "'enabled' must be a bool";
                    return result;
                }

                bool new_enabled = parameter.as_bool();
                if (new_enabled && !enabled_) {
                    q_iv2_ = q_iv_; // Snap target orientation to current orientation on enable
                    target_q_iv2_ = q_iv_;
                }
                enabled_ = new_enabled;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Attitude controller enabled: %s",
                    enabled_ ? "true" : "false"
                );
            }
            else if (parameter.get_name() == "integral_activation_threshold_deg")
            {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.successful = false;
                    result.reason = "'integral_activation_threshold_deg' must be a double";
                    return result;
                }

                integral_activation_threshold_deg_ = parameter.as_double();
                integral_activation_threshold_rad_ = integral_activation_threshold_deg_ * (M_PI / 180.0);
            }
            else if (parameter.get_name() == "max_slew_rate_roll_deg") {
                max_slew_rate_roll_deg_ = parameter.as_double();
                max_slew_rate_roll_rad_ = max_slew_rate_roll_deg_ * (M_PI / 180.0);
            }
            else if (parameter.get_name() == "max_slew_rate_pitch_deg") {
                max_slew_rate_pitch_deg_ = parameter.as_double();
                max_slew_rate_pitch_rad_ = max_slew_rate_pitch_deg_ * (M_PI / 180.0);
            }
            else if (parameter.get_name() == "max_slew_rate_yaw_deg") {
                max_slew_rate_yaw_deg_ = parameter.as_double();
                max_slew_rate_yaw_rad_ = max_slew_rate_yaw_deg_ * (M_PI / 180.0);
            }
            else if (parameter.get_name() == "P_ex") { P_ex_ = parameter.as_double(); }
            else if (parameter.get_name() == "P_ey") { P_ey_ = parameter.as_double(); }
            else if (parameter.get_name() == "P_ez") { P_ez_ = parameter.as_double(); }
            else if (parameter.get_name() == "I_ex") { I_ex_ = parameter.as_double(); }
            else if (parameter.get_name() == "I_ey") { I_ey_ = parameter.as_double(); }
            else if (parameter.get_name() == "I_ez") { I_ez_ = parameter.as_double(); }
            else if (parameter.get_name() == "P_wx") { P_wx_ = parameter.as_double(); }
            else if (parameter.get_name() == "P_wy") { P_wy_ = parameter.as_double(); }
            else if (parameter.get_name() == "P_wz") { P_wz_ = parameter.as_double(); }
            else if (parameter.get_name() == "I_MAX") { I_MAX_ = parameter.as_double(); }
            else if (parameter.get_name() == "buoyancy") { buoyancy_ = parameter.as_double(); }
            else if (parameter.get_name() == "r_bv_v") { r_bv_v_ = parameter.as_double_array(); }
        }

        // Reconstruct matrices with updated values
        P_e_ << P_ex_, 0, 0,
                0, P_ey_, 0,
                0, 0, P_ez_;
        
        I_e_ << I_ex_, 0, 0,
                0, I_ey_, 0,
                0, 0, I_ez_;
        
        P_w_ << P_wx_, 0, 0,
                0, P_wy_, 0,
                0, 0, P_wz_;

        return result;
    }

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto AttitudeControllerNode = std::make_shared<controls::AttitudeController>();
    rclcpp::spin(AttitudeControllerNode);
    rclcpp::shutdown();
    return 0;
}
