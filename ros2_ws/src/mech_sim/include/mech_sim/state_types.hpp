#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mech_sim {

using Vec3  = Eigen::Matrix<double,3,1>;
using Vec4  = Eigen::Matrix<double,4,1>;
using Vec6  = Eigen::Matrix<double,6,1>;
using Vec13 = Eigen::Matrix<double,13,1>;
using Mat3  = Eigen::Matrix<double,3,3>;
using Mat6  = Eigen::Matrix<double,6,6>;
using Mat4  = Eigen::Matrix<double,4,4>;

struct Params {
  Mat3 I   = Mat6::Identity();              // inertia (diag(Ixx,Iyy,Izz) to start)
  Mat6 D   = 0.0 * Mat6::Identity();        // start at zero, add damping later
  double mass = 28.0;                       // [kg] (redundant if M is filled)
  double dt   = 0.01;                      // [s] fixed step
  Vec3  gravity_odom(0.0,0.0,-9.81);        // odom-frame gravity dir
  double weight = mass * 9.81;              // [N] (set buoyancy separately)
  double buoyancy = mass * 1.05 * 9.81;     // [N] 5% buoyant. TODO: Change to more accurate model. 
  Vec3  r_gb_body(0.0,0.0,0.005);                // [m] vector from body origin to center of buoyancy (body frame) TODO: adjust to actual AUV after propulsion test
}
// State accessors (just views into the 13x1 state vector)
inline Vec3 get_v_body(const Vec13& x) { return x.segment<3>(0); }      // u v w
inline Vec3 get_w_body(const Vec13& x) { return x.segment<3>(3); }      // p q r
inline Vec4 get_q(const Vec13& x)      { return x.segment<4>(6); }      // qx qy qz qw
inline Vec3 get_p_odom(const Vec13& x) { return x.segment<3>(10); }     // px py pz

inline void set_v_body(Vec13& x, const Vec3& v){ x.segment<3>(0)=v; }
inline void set_w_body(Vec13& x, const Vec3& w){ x.segment<3>(3)=w; }
inline void set_q     (Vec13& x, const Vec4& q){ x.segment<4>(6)=q; }
inline void set_p_odom(Vec13& x, const Vec3& p){ x.segment<3>(10)=p; }

}