#include "plant/quad_dynamics_euler.h"

#include <cmath>

#include "common/math_util.h" // wrapToPi

namespace gnc {

QuadDynamics::QuadDynamics(const QuadParams& params)
  : params_(params),
    rng_(std::random_device{}()),
    norm_(0.0, 1.0)
{
}

void QuadDynamics::step(QuadStateTruth& state, double dt,const Vec4& bodyWrench) {
  // --- Params ---
  const double m  = params_.m;
  const double g  = gnc::GRAVITY;
  const double Ix = params_.Ix;
  const double Iy = params_.Iy;
  const double Iz = params_.Iz;

  // --- Unpack state ---
  const double vn = state.vel(0);
  const double ve = state.vel(1);
  const double vd = state.vel(2);

  const double phi   = state.euler(0);
  const double theta = state.euler(1);
  const double psi   = state.euler(2);

  const double p_rate = state.omega_b(0);
  const double q_rate = state.omega_b(1);
  const double r_rate = state.omega_b(2);

  // --- Inputs ---
  const double Fz  = bodyWrench(0); // thrust magnitude [N], positive
  const double Mx  = bodyWrench(1);
  const double My  = bodyWrench(2);
  const double Mz  = bodyWrench(3);

  // --- Rotation body->NED (same matrix form as MATLAB) ---
  Mat3 R_bn = RotB2N(phi, theta, psi);

  // --- Translational dynamics in NED ---
  // Gravity in NED is +down
  Vec3 g_ned;
  g_ned << 0.0, 0.0, g;

  // Thrust force in body frame for NED convention:
  // We want positive Fz to act "up", i.e. negative down direction.
  // So body force vector is [0;0;-Fz]
  Vec3 F_b;
  F_b << 0.0, 0.0, -Fz;

  // Acceleration
  Vec3 a_ned = (1.0 / m) * (R_bn * F_b) + g_ned;

  // Turbulence noise (per axis)
  Vec3 w;
  w << norm_(rng_), norm_(rng_), norm_(rng_);
  a_ned = a_ned + params_.sigma_turb.cwiseProduct(w);

  // --- Euler angle rates ---
  Vec3 omega;
  omega << p_rate, q_rate, r_rate;

  Vec3 euler_dot = eulerRates_ZYX(phi, theta, omega);

  // --- Rotational dynamics (diagonal inertia, matches MATLAB form) ---
  const double p_dot = (Mx + (Iy - Iz) * q_rate * r_rate) / Ix;
  const double q_dot = (My + (Iz - Ix) * p_rate * r_rate) / Iy;
  const double r_dot = (Mz + (Ix - Iy) * p_rate * q_rate) / Iz;

  // --- Assemble "xdot" pieces (in NED) ---
  Vec3 pos_dot;
  pos_dot << vn, ve, vd;

  Vec3 vel_dot = a_ned;

  Vec3 omega_dot;
  omega_dot << p_dot, q_dot, r_dot;

  // --- Integrate (explicit Euler) ---
  QuadStateTruth next = state;

  next.pos = next.pos + pos_dot * dt;
	if (next.pos(2) > 0) {
		next.pos(2) = 0;
	}
  next.vel = next.vel + vel_dot * dt;

  next.euler = next.euler + euler_dot * dt;
  next.omega_b = next.omega_b + omega_dot * dt;

  // Optional wrapping for nicer logs
  next.euler(0) = wrapToPi(next.euler(0));
  next.euler(1) = wrapToPi(next.euler(1));
  next.euler(2) = wrapToPi(next.euler(2));

  next.t = next.t + dt;

  state = next;
}

} // namespace gnc
