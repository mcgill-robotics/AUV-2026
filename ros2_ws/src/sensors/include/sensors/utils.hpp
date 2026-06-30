#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <stdexcept>
#include <tf2/utils.h>
#include <angles/angles.h>

namespace sensors::math {
using quatd = Eigen::Quaterniond;

inline quatd canonicalizeShortest(quatd q) {
  // Same rotation, consistent representation (avoid >180° path in interpolation/control)
  if (q.w() < 0.0) q.coeffs() *= -1.0;  // coeffs = (x,y,z,w) in Eigen
  return q;
}

inline quatd quatFromParamWxyz(const std::vector<double>& v) {
  if (v.size() != 4) throw std::runtime_error("Quaternion param must be [w,x,y,z]");
  quatd q(v[0], v[1], v[2], v[3]);
  return q.normalized();
}

inline double yawFromQuat(const quatd& q) {
    tf2::Quaternion tf2_q(q.x(), q.y(), q.z(), q.w());
    return tf2::getYaw(tf2_q);
}

inline double normalizeAngle(double angle) {
    return angles::normalize_angle(angle);
}

} // namespace sensors::math

// Courtesy of Ben Hepditch, https://github.com/ben-jamming 
