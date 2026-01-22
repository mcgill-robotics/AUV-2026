#include "sensors/Imu_processor.hpp"

namespace sensors
{
ImuProcessor::ImuProcessor()
    : Node("imu_processor"), q_vi_(quatd::Identity())
{

    imu_pub_ = this->create_publisher<imu_msg>(
        "processed/imu",
        10
    );
    imu_sub_ = this->create_subscription<imu_msg>(
        "imu/data",
        10,
        std::bind(&ImuProcessor::imu_callback, this, std::placeholders::_1)
    );

    this->declare_parameter<std::vector<double>>("q_sv", {1, 0.0, 0.0, 0.0}); // Default no rotation. 
    this->declare_parameter<std::vector<double>>("q_in", {1.0, 0.0, 0.0, 0.0}); // Default: no rotation
    
    std::vector<double> q_sv_vec;
    std::vector<double> q_in_vec;
    this->get_parameter("q_sv", q_sv_vec);
    this->get_parameter("q_in", q_in_vec);

    q_sv_ = quatd(q_sv_vec[0], q_sv_vec[1], q_sv_vec[2], q_sv_vec[3]);
    q_in_ = quatd(q_in_vec[0], q_in_vec[1], q_in_vec[2], q_in_vec[3]);

    // Gravity vector in inertial frame (down is negative z)
    g_i << 0.0, 0.0, -9.81;

    q_vs_ = q_sv_.inverse();
}

Vec3 ImuProcessor::compute_free_acc(const Vec3& specific_force, const quatd& q_si) const
{
    // Rotate gravity into sensor frame
    Vec3 g_s = q_si * g_i; // This is an overload of the quaternion operator* for vectors.

    // Free acceleration 
    Vec3 a_free = q_vs_ * (specific_force + g_s);
    return a_free;
}

Vec3 ImuProcessor::rotate_gyro(const Vec3& w_s) const
{
    // Rotate gyro measurements into vehicle frame
    Vec3 w_v = q_vs_ * w_s;
    return w_v;
}

void ImuProcessor::imu_callback(const imu_msg::SharedPtr imu_in) 
{
    imu_msg imu_out = *imu_in;
    imu_out.header.frame_id = "auv"; // All data will be processed into the AUV frame

    // Specific force vector from IMU
    Vec3 f_s;
    f_s << imu_in->linear_acceleration.x, imu_in->linear_acceleration.y, imu_in->linear_acceleration.z;   

    // Orientation remains unchanged but rotated to AUV frame
    quatd q_is(
        imu_in->orientation.w,
        imu_in->orientation.x,
        imu_in->orientation.y,
        imu_in->orientation.z
    ); 
    
    // Compute free acceleration
    Vec3 a_free = compute_free_acc(f_s, q_is.inverse());
    imu_out.linear_acceleration.x = a_free(0);
    imu_out.linear_acceleration.y = a_free(1); 
    imu_out.linear_acceleration.z = a_free(2);


    q_iv_ = q_is_ * q_sv_; // New orientation: world frame relative to vehicle frame

    if (q_iv_.w() < 0) {
        q_iv_.coeffs() *= -1.0; // Ensure positive scalar part for consistency
    }

    imu_out.orientation.w = q_iv_.w();
    imu_out.orientation.x = q_iv_.x();
    imu_out.orientation.y = q_iv_.y();
    imu_out.orientation.z = q_iv_.z();

    // Angular Velocity
    Vec3 w_s = Vec3(
        imu_in->angular_velocity.x,
        imu_in->angular_velocity.y,
        imu_in->angular_velocity.z
    );
    Vec3 w_v = rotate_gyro(w_s);
    imu_out.angular_velocity.x = w_v(0);
    imu_out.angular_velocity.y = w_v(1);
    imu_out.angular_velocity.z = w_v(2);

    // Publish processed message
    imu_pub_->publish(imu_out);
}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sensors::ImuProcessor>());
	rclcpp::shutdown();

	return 0;
}