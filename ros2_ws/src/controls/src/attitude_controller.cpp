#include "controls/attitude_controller.hpp"

namespace controls
{
    AttitudeController::AttitudeController(): Node("attitude_controller")
    {
        this->declare_parameter<double>("P_ex", 1.0);
        this->declare_parameter<double>("P_ey", 1.0);
        this->declare_parameter<double>("P_ez", 1.0);
        this->declare_parameter<double>("P_wx", 1.0);
        this->declare_parameter<double>("P_wy", 1.0);
        this->declare_parameter<double>("P_wz", 1.0);
        this->declare_parameter<double>("buoyancy", 278.0); // Newtons
        this->declare_parameter<std::vector<double>>("r_bv", {0.0, 0.0, 0.023}); // [m] From CAD Model
        this->declare_parameter<double>("control_loop_hz", 10.0); // Control loop frequency

        this->get_parameter("P_ex", P_ex_);
        this->get_parameter("P_ey", P_ey_);
        this->get_parameter("P_ez", P_ez_);
        this->get_parameter("P_wx", P_wx_);
        this->get_parameter("P_wy", P_wy_);
        this->get_parameter("P_wz", P_wz_);
        this->get_parameter("buoyancy", buoyancy_);
        this->get_parameter("r_bv", r_bv_);
        this->get_parameter("control_loop_hz", control_loop_hz_);

        q_iv_ = quatd::Identity(); // Initial orientation: identity quaternion
        w_iv_ = Vec3::Zero(); // Initial angular velocity: zero vector
        q_iv2_ = quatd::Identity(); // Initial target orientation: identity quaternion

        P_e_ << P_ex_, 0, 0,
                0, P_ey_, 0,
                0, 0, P_ez_;
        
        P_w_ << P_wx_, 0, 0,
                0, P_wy_, 0,
                0, 0, P_wz_;


        pub_effort_ = this->create_publisher<wrench_msg>("/controls/attitude_effort", 1);
        sub_imu_ = this->create_subscription<imu_msg>(
            "auv_frame/imu",
            1,
            std::bind(&AttitudeController::imu_callback, this, std::placeholders::_1)
        );
        sub_target_orientation_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/controls/quaternion_setpoint",
            1,
            std::bind(&AttitudeController::target_orientation_callback, this, std::placeholders::_1)
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
        q_iv2_ = quatd(
            msg->w,
            msg->x,
            msg->y,
            msg->z
        );
    }


    Vec3 AttitudeController::feedback_effort(const quatd& q_iv2)
    {
        quatd q_error = q_iv_.conjugate() * q_iv2;
        Vec3 error_vector = Vec3(q_error.x(), q_error.y(), q_error.z());
        Vec3 feedback = P_e_ * error_vector - P_w_ * w_iv_; //TODO: Verify sign conventions
        return feedback;
    }

    Vec3 AttitudeController::feedforward_effort()
    {
        // Compute the torque due to buoyancy offset
        Vec3 r_bv_vec(r_bv_[0], r_bv_[1], r_bv_[2]);
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
        wrench_msg effort = compute_control_effort();
        pub_effort_->publish(effort);
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
