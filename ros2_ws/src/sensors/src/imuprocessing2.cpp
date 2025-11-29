#include "imuprocessing2.hpp" 

// Constructor Implementation
PoolIMUProcessor::PoolIMUProcessor()
{
    // Gravity in earth frame (Z-up).
    gravity_vector_earth_ << 0.0, 0.0, -9.81;

    earth_to_pool_rotation_set_ = false;

    // Replace with CAD-derived fixed rotation.
    R_sensor_to_vehicle_.setIdentity();
}

// handleZeroRequest Implementation
void PoolIMUProcessor::handleZeroRequest()
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

    // Convert averaged quaternion -> rotation matrix
    Eigen::Matrix3d R_pool_to_earth = quaternionToMatrix(avg_q);

    // Earth -> pool = transpose
    R_earth_to_pool_ = R_pool_to_earth.transpose();

    earth_to_pool_rotation_set_ = true;
}

// processIMUData Implementation
Eigen::Vector3d PoolIMUProcessor::processIMUData(
    const Eigen::Vector4d& live_orientation_quat,
    const Eigen::Vector3d& accel_sensor_frame,
    const Eigen::Vector3d& /*gyro_sensor_frame*/)
{
    if (!earth_to_pool_rotation_set_)
    {
        return Eigen::Vector3d::Zero();
    }

    // (1) sensor -> vehicle
    Eigen::Vector3d accel_vehicle_frame = R_sensor_to_vehicle_ * accel_sensor_frame;

    // (2) sensor -> earth (from quaternion)
    Eigen::Matrix3d R_sensor_to_earth = quaternionToMatrix(live_orientation_quat);

    // (3) vehicle -> sensor
    Eigen::Matrix3d R_vehicle_to_sensor = R_sensor_to_vehicle_.transpose();

    // (4) vehicle -> earth
    Eigen::Matrix3d R_vehicle_to_earth = R_sensor_to_earth * R_vehicle_to_sensor;

    Eigen::Vector3d accel_earth_frame = R_vehicle_to_earth * accel_vehicle_frame;

    // (5) specific force -> motion accel
    Eigen::Vector3d final_accel_earth = accel_earth_frame + gravity_vector_earth_;

    // (6) earth -> pool
    Eigen::Vector3d accel_pool_frame = R_earth_to_pool_ * final_accel_earth;

    return accel_pool_frame;
}

// setSensorToVehicleRotation Implementation
void PoolIMUProcessor::setSensorToVehicleRotation(const Eigen::Matrix3d& R)
{
    R_sensor_to_vehicle_ = R;
}

// quaternionToMatrix Implementation (Stub)
Eigen::Matrix3d PoolIMUProcessor::quaternionToMatrix(const Eigen::Vector4d& q) const
{
    (void)q; // Prevent unused variable warning
    return Eigen::Matrix3d::Identity(); // placeholder
}

// getLiveIMUQuaternion Implementation (Stub)
Eigen::Vector4d PoolIMUProcessor::getLiveIMUQuaternion()
{
    return Eigen::Vector4d(1,0,0,0); // identity rotation
}