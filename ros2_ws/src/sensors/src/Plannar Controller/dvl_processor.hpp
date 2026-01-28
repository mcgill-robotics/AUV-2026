#include "sensors/Imu_processor.hpp" // For types like Vec3 and quatd

namespace sensors {
class DvlProcessor {
public:
    // r_sd_v: Vector from DVL Sensor to Vehicle CM in Body Frame
    DvlProcessor(const Vec3& r_sd_v);

    // process: Takes DVL position and IMU orientation, returns CM position
    Vec3 process(const Vec3& p_dvl_i, const quatd& q_vi) const;

private:
    Vec3 r_sd_v_; // Stored offset from SolidWorks
};
}