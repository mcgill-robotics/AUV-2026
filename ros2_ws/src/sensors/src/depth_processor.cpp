#include "sensors/depth_processor.hpp"

namespace sensors
{
DepthRepublisher::DepthRepublisher(const Vec3& r_vs_v)
    : r_vs_v_(r_vs_v)
{
};

float64_msg DepthRepublisher::process(const float64_msg& depth_in, const quatd& q_vi) const
{
    Vec3 r_vs_i = q_vi.inverse() * r_vs_v_;
    double r_vi_i = depth_in.data + r_vs_i(2); // Add z-component of r_vs_i to depth measurement
    float64_msg depth_out ;
    depth_out.data = r_vi_i;

    return depth_out;

} 
}