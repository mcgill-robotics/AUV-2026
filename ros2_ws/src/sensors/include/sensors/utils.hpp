#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <stdexcept>

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
} // namespace sensors::math

// Courtesy of Ben Hepditch, https://github.com/ben-jamming 
