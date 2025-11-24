#pragma once
#include <Eigen/Dense>
#include <vector>
#include <stdexcept>

class PoolIMUProcessor
{
public:
    PoolIMUProcessor()
    {
        // Gravity in earth frame (Z-up).
        gravity_vector_earth_ << 0.0, 0.0, -9.81;

        earth_to_pool_rotation_set_ = false;

        // Replace with CAD-derived fixed rotation.
        R_sensor_to_vehicle_.setIdentity();
    }

    // ------------------------------------------------------------
    // Zeroing: hold AUV still, collect ~50 orientation samples
    // to define pool frame relative to earth frame.
    // ------------------------------------------------------------
    void handleZeroRequest()
    {
        const int NUM_SAMPLES = 50;

        Eigen::Vector4d quat_accum(0,0,0,0);

        for (int i = 0; i < NUM_SAMPLES; ++i)
        {
            Eigen::Vector4d q = getLiveIMUQuaternion();
            quat_accum += q;   // simple sum (no normalization)
        }

        // Simple average
        Eigen::Vector4d avg_q = quat_accum / static_cast<double>(NUM_SAMPLES);

        // Convert averaged quaternion → rotation matrix
        Eigen::Matrix3d R_pool_to_earth = quaternionToMatrix(avg_q);

        // Earth → pool = transpose
        R_earth_to_pool_ = R_pool_to_earth.transpose();

        earth_to_pool_rotation_set_ = true;
    }

    // Main processing function
    Eigen::Vector3d processIMUData(
        const Eigen::Vector4d& live_orientation_quat,
        const Eigen::Vector3d& accel_sensor_frame,
        const Eigen::Vector3d& /*gyro_sensor_frame*/)
    {
        if (!earth_to_pool_rotation_set_)
        {
            return Eigen::Vector3d::Zero();
        }

        // (1) sensor → vehicle
        Eigen::Vector3d accel_vehicle_frame = R_sensor_to_vehicle_ * accel_sensor_frame;

        // (2) sensor → earth (from quaternion)
        Eigen::Matrix3d R_sensor_to_earth = quaternionToMatrix(live_orientation_quat);

        // (3) vehicle → sensor
        Eigen::Matrix3d R_vehicle_to_sensor = R_sensor_to_vehicle_.transpose();

        // (4) vehicle → earth
        Eigen::Matrix3d R_vehicle_to_earth = R_sensor_to_earth * R_vehicle_to_sensor;

        Eigen::Vector3d accel_earth_frame = R_vehicle_to_earth * accel_vehicle_frame;

        // (5) specific force → motion accel
        Eigen::Vector3d final_accel_earth = accel_earth_frame + gravity_vector_earth_;

        // (6) earth → pool
        Eigen::Vector3d accel_pool_frame = R_earth_to_pool_ * final_accel_earth;

        return accel_pool_frame;
    }

    void setSensorToVehicleRotation(const Eigen::Matrix3d& R)
    {
        R_sensor_to_vehicle_ = R;
    }

private:
    // Stub: quaternion (w,x,y,z) → rotation matrix
    Eigen::Matrix3d quaternionToMatrix(const Eigen::Vector4d& q) const
    {
        (void)q;
        return Eigen::Matrix3d::Identity(); // placeholder
    }

    // Stub: get IMU quaternion
    Eigen::Vector4d getLiveIMUQuaternion()
    {
        return Eigen::Vector4d(1,0,0,0); // identity rotation
    }

private:
    Eigen::Vector3d gravity_vector_earth_;

    Eigen::Matrix3d R_earth_to_pool_;
    bool earth_to_pool_rotation_set_;

    Eigen::Matrix3d R_sensor_to_vehicle_;
};
