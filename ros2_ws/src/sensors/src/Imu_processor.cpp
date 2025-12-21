#include "sensors/Imu_processor.hpp"

namespace sensors
{
ImuRepublisher::ImuRepublisher(const quatd& q_sv, const quatd& q_in)
    : q_vi_(quatd::Identity()), q_sv_(q_sv), q_in_(q_in)
{
    // Gravity vector in inertial frame (down is negative z)
    g_i << 0.0, 0.0, -9.81;
}

Vec3 ImuRepublisher::compute_free_acc(const Vec3& specific_force, const quatd& q_si) const
{
    // Rotate gravity into sensor frame
    Vec3 g_s = q_si * g_i; // This is an overload of the quaternion operator* for vectors.

    // Free acceleration 
    Vec3 a_free = q_sv_.inverse() * (specific_force - g_s);
    return a_free;
}

Vec3 ImuRepublisher::rotate_gyro(const Vec3& w_s, const quatd& q_sv) const
{
    // Rotate gyro measurements into vehicle frame
    Vec3 w_v = q_sv.inverse() * w_s;
    return w_v;
}

imu_msg ImuRepublisher::process(const imu_msg& imu_in) 
{
    imu_msg imu_out = imu_in; // Copy input to output
    imu_out.header.frame_id = "auv"; // All data will be processed into the AUV frame

    // Specific force vector from IMU
    Vec3 f_s;
    f_s << imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z;   
    
    // Compute free acceleration
    Vec3 a_free = compute_free_acc(f_s, q_in_);
    imu_out.linear_acceleration.x = a_free(0);
    imu_out.linear_acceleration.y = a_free(1); 
    imu_out.linear_acceleration.z = a_free(2);

    // Orientation remains unchanged but rotated to AUV frame
    quatd q_si(
        imu_in.orientation.w,
        imu_in.orientation.x,
        imu_in.orientation.y,
        imu_in.orientation.z
    ); // Assumption: IMU messages report world frame relative to sensor. 

    q_vi_ = q_sv_.inverse() * q_si; // New orientation: world frame relative to vehicle frame
    imu_out.orientation.w = q_vi_.w();
    imu_out.orientation.x = q_vi_.x();
    imu_out.orientation.y = q_vi_.y();
    imu_out.orientation.z = q_vi_.z();

    // Angular Velocity
    Vec3 w_s = Vec3(
        imu_in.angular_velocity.x,
        imu_in.angular_velocity.y,
        imu_in.angular_velocity.z
    );
    Vec3 w_v = rotate_gyro(w_s, q_sv_);
    imu_out.angular_velocity.x = w_v(0);
    imu_out.angular_velocity.y = w_v(1);
    imu_out.angular_velocity.z = w_v(2);

    // Return processed message
    return imu_out;
}
}