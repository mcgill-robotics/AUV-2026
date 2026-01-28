#include "sensors/dvl_processor.hpp"

namespace sensors {

DvlProcessor::DvlProcessor(const Vec3& r_sd_v)
    : r_sd_v_(r_sd_v) 
{
    // Constructor simply saves the SolidWorks offset
}

Vec3 DvlProcessor::process(const Vec3& p_dvl_i, const quatd& q_vi) const {
    // 1. Rotate the body-frame offset into the pool frame
    // This is the 'R' part of your transformation matrix
    Vec3 r_sd_i = q_vi.inverse() * r_sd_v_;

    // 2. Add the rotated offset to the DVL pool position
    // This is the 'Translation' part of your matrix
    Vec3 p_vi_i = p_dvl_i + r_sd_i;

    return p_vi_i; // This is the CM position in the pool frame
}

}