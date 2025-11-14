#pragma once
// dynamics_core.hpp
//
// Purpose: Declare the core physics class for your 13-state AUV model.
// State x = [ u v w  p q r  q0 qx qy qz  px py pz ]^T
//            (body lin vel) (body ang vel) (quat body->odom) (position in odom)
//
// This header only declares the API. Implementations live in dynamics_core.cpp.

#include "mech_sim/state_types.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace auv_sim {

/// DynamicsCore13 implements f(x, tau) and a simple fixed-step integrator (RK4).
/// - x: 13x1 state (see layout above)
/// - tau_body: 6x1 body wrench [Fx Fy Fz Mx My Mz]^T
/// - Params: inertia (I), damping (D), weight/buoyancy, dt, etc.
class DynamicsCore {
public:
  /// Construct with physical parameters (copied internally).
  /// 
  explicit DynamicsCore(const Params& params);

  /// Right-hand side of the ODE: xdot = f(x, tau_body).
  /// Safe to call at any rate (no internal state).
  Vec13 f(const Vec13& x, const Vec6& tau_body) const;

  /// Advance one step with classic Runge-Kutta 4 (fixed dt you pass in).
  /// Renormalizes the quaternion on output.
  Vec13 step_rk4(const Vec13& x, const Vec6& tau_body, double dt) const;


private:
  // ---- Helpers used by f() ----

  /// Compute body accelerations: nudot = M^{-1}( tau - D*nu - gb )
  /// where nu = [u v w p q r]^T and gb is gravity/buoyancy wrench in body.
  Vec6 nu_dot(const Vec13& x, const Vec6& tau_body) const;

  /// Net gravity+buoyancy force and moment expressed in the BODY frame.
  /// Returns a 6x1 wrench [Fx Fy Fz Mx My Mz]^T.
  Vec6 gravity_buoyancy_wrench_body(const Mat3 R_body_to_odom) const;

  /// Convert quaternion vector (qx,qy,qz,qw) to Eigen::Quaterniond (w,x,y,z).
  static Eigen::Quaterniond q_from_vec(const Vec4& qv);

  /// Rotation matrix body->odom from quaternion.
  static Mat3 R_body_to_odom(const Vec4& q);

  /// Ω(ω) used in quaternion kinematics: qdot = 0.5 * Ω(ω_body) * q
  static Mat4 Omega(const Vec3& w_body);

private:
  Params P_;      // physical parameters
  Mat3   Iinv_;   // inverse(I) cached once in ctor
};

} // namespace auv_sim