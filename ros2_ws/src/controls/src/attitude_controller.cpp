#include "controls/attitude_controller.hpp"
#include "sensors/utils.hpp"

namespace controls
{
    AttitudeController::AttitudeController(): Node("attitude_controller")
    {
        // Large-error attitude controller gains
        this->declare_parameter<double>("P_ex_large", 1.0);
        this->declare_parameter<double>("P_ey_large", 1.0);
        this->declare_parameter<double>("P_ez_large", 1.0);
        this->declare_parameter<double>("P_wx_large", 1.0);
        this->declare_parameter<double>("P_wy_large", 1.0);
        this->declare_parameter<double>("P_wz_large", 1.0);

        // SAS / small-error attitude hold gains
        this->declare_parameter<double>("P_ex_sas", 0.5);
        this->declare_parameter<double>("P_ey_sas", 0.5);
        this->declare_parameter<double>("P_ez_sas", 0.5);
        this->declare_parameter<double>("P_wx_sas", 2.0);
        this->declare_parameter<double>("P_wy_sas", 2.0);
        this->declare_parameter<double>("P_wz_sas", 2.0);

        this->declare_parameter<double>("buoyancy", 278.0); // [N]
        this->declare_parameter<std::vector<double>>("r_bv_v", {0.0, 0.0, 0.023}); // [m] From CAD model
        this->declare_parameter<double>("control_loop_hz", 100.0); //Control loop frequency

        this->declare_parameter<double>("sas_switch_deg", 5.0);

        this->declare_parameter<bool>("enabled", false);

        this->get_parameter("P_ex_large", P_ex_large_);
        this->get_parameter("P_ey_large", P_ey_large_);
        this->get_parameter("P_ez_large", P_ez_large_);
        this->get_parameter("P_wx_large", P_wx_large_);
        this->get_parameter("P_wy_large", P_wy_large_);
        this->get_parameter("P_wz_large", P_wz_large_);

        this->get_parameter("P_ex_sas", P_ex_sas_);
        this->get_parameter("P_ey_sas", P_ey_sas_);
        this->get_parameter("P_ez_sas", P_ez_sas_);
        this->get_parameter("P_wx_sas", P_wx_sas_);
        this->get_parameter("P_wy_sas", P_wy_sas_);
        this->get_parameter("P_wz_sas", P_wz_sas_);

        this->get_parameter("buoyancy", buoyancy_); 
        this->get_parameter("r_bv_v", r_bv_v_);
        this->get_parameter("control_loop_hz", control_loop_hz_);
        this->get_parameter("sas_switch_deg", sas_switch_deg_);
        this->get_parameter("enabled", enabled_);

        sas_switch_ = sas_switch_deg_ * M_PI / 180.0;

        q_iv_ = quatd::Identity(); // Initial orientation: identity quaternion
        w_iv_v = Vec3::Zero(); // Initial angular velocity: zero vector
        q_iv2_ = quatd::Identity(); // Initial target orientation: identity quaternion

        P_e_large_ << P_ex_large_, 0, 0,
                    0, P_ey_large_, 0,
                    0, 0, P_ez_large_;

        P_w_large_ << P_wx_large_, 0, 0,
                    0, P_wy_large_, 0,
                    0, 0, P_wz_large_;

        P_e_sas_ << P_ex_sas_, 0, 0,
                    0, P_ey_sas_, 0,
                    0, 0, P_ez_sas_;

        P_w_sas_ << P_wx_sas_, 0, 0,
                    0, P_wy_sas_, 0,
                    0, 0, P_wz_sas_;


        pub_effort_ = this->create_publisher<wrench_msg>("/controls/attitude_effort", rclcpp::SensorDataQoS().keep_last(1));
        sub_imu_ = this->create_subscription<imu_msg>(
            "auv_frame/imu",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&AttitudeController::imu_callback, this, std::placeholders::_1)
        );
        sub_attitude_reference_ = this->create_subscription<auv_msgs::msg::AttitudeReference>(
            "/controls/attitude_reference",
            1,
            std::bind(&AttitudeController::target_attitude_callback, this, std::placeholders::_1)
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
        w_iv_v = Vec3(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
    }

    void AttitudeController::target_attitude_callback(const auv_msgs::msg::AttitudeReference::SharedPtr msg)
    {
        q_iv2_ = quatd(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z
        );
        w_ref_v = Vec3(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
    }


    Vec3 AttitudeController::feedback_effort(const quatd& q_error, const ControlMode& mode)
    {

        Vec3 feedback = Vec3::Zero();

        switch(mode)
        {
            case ControlMode::LARGE_ERROR:
            {
                double theta = 2.0 * std::acos(std::clamp(q_error.w(), -1.0, 1.0));
                Vec3 axis_error = q_error.vec().normalized();
                Vec3 error_vec = theta * axis_error;
                feedback = P_e_large_ * error_vec - P_w_large_ * (w_iv_v - w_ref_v);
                break;
            }
            case ControlMode::SAS:
            {
                feedback = P_e_sas_ * Vec3(q_error.x(), q_error.y(), q_error.z()) - P_w_sas_ * w_iv_v; // Small-angle approximation: use vector part of quaternion error directly
                break;
            }
        }
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
        quatd q_error = q_iv_.conjugate() * q_iv2_;
        q_error = sensors::math::canonicalizeShortest(q_error);
        double q_w = std::clamp(q_error.w(), -1.0, 1.0);
        double angle_error = 2.0 * std::acos(q_w);
        ControlMode mode;

        if (angle_error < sas_switch_)
        {
            mode = ControlMode::SAS;
        }
        else
        {
            mode = ControlMode::LARGE_ERROR;
        }

        Vec3 feedback = feedback_effort(q_error, mode);
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
        wrench_msg effort;
        if (enabled_)
        {
            effort = compute_control_effort();
        }
        else
        {
            effort.force.x = 0.0;
            effort.force.y = 0.0;
            effort.force.z = 0.0;
            effort.torque.x = 0.0;
            effort.torque.y = 0.0;
            effort.torque.z = 0.0;
        }

        pub_effort_->publish(effort);
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

                enabled_ = parameter.as_bool();
                RCLCPP_INFO(
                    this->get_logger(),
                    "Attitude controller enabled: %s",
                    enabled_ ? "true" : "false"
                );
            }
        }

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
