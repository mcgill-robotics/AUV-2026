#pragma once
#include <Eigen/Dense>
#include <vector>
#include <stdexcept>

class PoolIMUProcessor
{
public:
    // Constructor
    PoolIMUProcessor();

    // Zeroing: hold AUV still, collect ~50 orientation samples
    void handleZeroRequest();

    // Main processing function
    Eigen::Vector3d processIMUData(
        const Eigen::Vector4d& live_orientation_quat,
        const Eigen::Vector3d& accel_sensor_frame,
        const Eigen::Vector3d& /*gyro_sensor_frame*/); // Argument commented out to match original logic

    // Setter for rotation matrix
    void setSensorToVehicleRotation(const Eigen::Matrix3d& R);

private:
    // Helper: quaternion (w,x,y,z) -> rotation matrix
    Eigen::Matrix3d quaternionToMatrix(const Eigen::Vector4d& q) const;

    // Helper: get IMU quaternion
    Eigen::Vector4d getLiveIMUQuaternion();

private:
    Eigen::Vector3d gravity_vector_earth_;

    Eigen::Matrix3d R_earth_to_pool_;
    bool earth_to_pool_rotation_set_;

    Eigen::Matrix3d R_sensor_to_vehicle_;
};