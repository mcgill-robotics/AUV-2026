#include "mech_sim/dynamics_core.hpp"
#include "mech_sim/state_types.hpp"

// main physics engine behind the mech sim AUV model. We solve a 13-state ODE
// with RK4 integration. The state is:
// x = [ u v w        p q r         qw qx qy qz         px py pz ]^T
// (body lin vel) (body ang vel) (quat body->odom) (position in odom)
// The inputs are body-frame wrenches: tau_body = [Fx Fy Fz Mx My Mz]^T
// Main resources used: "Marine Control Systems: Guidance, Navigation and Control of Ships, Rigs and Underwater Vehicles" by Thor I. Fossen.
// https://www.bartslinger.com/flight-dynamics/equations-of-motion-with-quaternions 


namespace mech_sim {
DynamicsCore::DynamicsCore(const Params& params)
  : P_(params)
{
 // Precompute inverse of inertia matrix I
 Iinv_ = P_.I.inverse();

}

 // Compute Rotation matrix R from quaternion. 
 Mat3 DynamicsCore::R_from_q(const Vec4& q) { //assume q = (qw,qx,qy,qz)
   quatd quat(q(0), q(1), q(2), q(3)); // w, x, y, z
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
   Vec6 DynamicsCore::gravity_buoyancy_wrench_body(const Mat3& R_body_to_odom, const double depth) const {

        // Gravity force in odom frame
        Vec3 force_g_odom(0.0,0.0,-P_.mass * 9.81);
        // Buoyancy force in odom frame
        double buoyancy;
        if (depth <= 0.0) {
          buoyancy = P_.buoyancy * (1 + (depth / P_.H));
          buoyancy = std::max(0.0, buoyancy);
        }
        else {
          buoyancy = P_.buoyancy;
        };

        Vec3 force_b_odom(0.0,0.0,buoyancy);

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

        Vec6 DynamicsCore::damping_wrench_body(const Vec3& v_body, const Vec3& w_body) const {
          Vec6 wrench;
          const double rho = P_.rho;
          const double L = P_.L;
          const double B = P_.W;
          const double H = P_.H;

          // Quadratic damping Coefficients
          double X_uu = 0.5 * rho * P_.Cd_x * (P_.W * P_.H);
          double Y_vv = 0.5 * rho * P_.Cd_y * (P_.L * P_.H);
          double Z_ww = 0.5 * rho * P_.Cd_z * (P_.L * P_.W);

          // Yaw quadratic damping coefficient N_{|r|r}
          double N_rr = (rho * P_.Cd_y * H * std::pow(L, 4)) / 64.0;

          // Roll quadratic damping coefficient K_{|p|p}
          double K_pp = (rho * P_.Cd_z * B * std::pow(H, 3)) / 32.0;

          // Pitch quadratic damping coefficient M_{|q|q}
          double M_qq = (rho * P_.Cd_z * L * std::pow(H, 3)) / 32.0;

          // Compute damping forces
          double X_damp = -X_uu * std::abs(v_body(0)) * v_body(0);
          double Y_damp = -Y_vv * std::abs(v_body(1)) * v_body(1);
          double Z_damp = -Z_ww * std::abs(v_body(2)) * v_body(2);  
          double K_damp = -K_pp * std::abs(w_body(0)) * w_body(0);
          double M_damp = -M_qq * std::abs(w_body(1)) * w_body(1);
          double N_damp = -N_rr * std::abs(w_body(2)) * w_body(2);  

          // Assemble wrench
          wrench.segment<6>(0) = Vec6(X_damp, Y_damp, Z_damp, K_damp, M_damp, N_damp);

          return wrench;
        }

        Vec6 DynamicsCore::nu_dot(const Vec13& x, const Vec6& tau_body) const {
          // Extract gravity and buoyancy wrench in body frame
          Mat3 R_body_to_odom = R_from_q( get_q(x) );
          Vec6 gb_wrench_body = gravity_buoyancy_wrench_body(R_body_to_odom, get_p_odom(x)(2) );

          // Extract Body thrust forces and moments
          Vec3 force_body = tau_body.segment<3>(0);
          Vec3 moment_body = tau_body.segment<3>(3);

          // Compute damping wrench
          Vec3 v_body = get_v_body(x);
          Vec3 w_body = get_w_body(x);
          Vec6 damping_wrench = damping_wrench_body(v_body, w_body);

          //Net forces and moments
          Vec3 net_force = force_body + gb_wrench_body.segment<3>(0) + damping_wrench.segment<3>(0);
          Vec3 net_moment = moment_body + gb_wrench_body.segment<3>(3) + damping_wrench.segment<3>(3);

          Vec3 v_dot = (1.0 / P_.mass) * net_force - w_body.cross(v_body);
          Vec3 w_dot = Iinv_ * (net_moment - w_body.cross(P_.I * w_body) ); 

          Vec6 nu_dot;
          nu_dot.segment<3>(0) = v_dot;
          nu_dot.segment<3>(3) = w_dot;
          return nu_dot;

        }

    Vec4 DynamicsCore::q_dot(const Vec4& q, const Vec3& w_body) {
        Mat4 Om = Omega(w_body);
        Vec4 qd = 0.5 * Om * q;
        return qd;
    }

    Vec3 DynamicsCore::p_odom_dot(const Vec4& q, const Vec3& v_body) {
        Mat3 R_body_to_odom = R_from_q(q);
        Vec3 p_dot = R_body_to_odom * v_body;
        return p_dot;
    }

} // namespace auv_sim