#include "mech_sim/dynamics_core.hpp"
#include "mech_sim/state_types.hpp"

// main physics engine behing the mech sim AUV model. We solve a 13-state ODE
// with RK4 integration. The state is:
// x = [ u v w  p q r  q0 qx qy qz  px py pz ]^T
// (body lin vel) (body ang vel) (quat body->odom) (position in odom)
// The inputs are body-frame wrenches: tau_body = [Fx Fy Fz Mx My Mz]^T
// Main resources used: "Marine Control Systems: Guidance, Navigation and Control of Ships, Rigs and Underwater Vehicles" by Thor I. Fossen.
// https://www.bartslinger.com/flight-dynamics/equations-of-motion-with-quaternions 


namespace auv_sim {
DynamicsCore::DynamicsCore(const Params& params)
  : P_(params)
{
 // Precompute inverse of inertia matrix I
 Iinv_ = P_.I.inverse();


 // Compute Rotation matrix R from quaternion. 
 Mat3 DynamicsCore::R_body_to_odom(const Vec4& q) { //asume q = (qw,qx,qy,qz)
   Eigen::Quaterniond quat(q(0), q(1), q(2), q(3)); // w, x, y, z
   return quat.toRotationMatrix();
 }

   Mat4 DynamicsCore::Omega(const Vec3& w_body) {
        const double p = w_body[0], q = w_body[1], r = w_body[2];
        Mat4 Om;
        Om <<    0, -p, -q, -r,
                 p,  0,  r, -q,
                 q, -r,  0,  p,
                 r,  q, -p,  0;
         return Om;
   }

   // Compute gravity and buoyancy wrench in body frame
   Vec6 DynamicsCore::gravity_buoyancy_wrench_body(const Mat3 R_body_to_odom) const {

   // Gravity force in odom frame
        Vec3 force_g_odom(0.0,0.0,-P_.weight);
        // Buoyancy force in odom frame
        Vec3 force_b_odom(0.0,0.0,-P_.buoyancy);

        // Transform forces to body frame
        Mat3 R_odom_to_body = R_body_to_odom.transpose();

        // Forces in body frame
        Vec3 force_g_body = R_odom_to_body * force_g_odom;
        Vec3 force_b_body = R_odom_to_body * force_b_odom;

        // Hydrostatic torque due to offset of center of buoyancy
        Vec3 torque_b_body = P_.r_gb_body.cross(force_b_body);

        // Net wrench
        Vec6 wrench;
        wrench.segment<3>(0) = force_g_body + force_b_body; // Net force
        wrench.segment<3>(3) = torque_b_body;

        return wrench; }

}

} // namespace auv_sim