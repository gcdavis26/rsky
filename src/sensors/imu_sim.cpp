#include "sensors/imu_sim.h"

#include <cmath>

namespace gnc {

ImuSim::ImuSim(const QuadParams& quad, const ImuSimParams& p, uint32_t seed)
  : quad_(quad),
    p_(p),
    noise_(seed) {
  reset();
}

void ImuSim::reset() {
  has_prev_ = false;
  v_prev_ned_.setZero();
}

ImuMeas ImuSim::step(const QuadStateTruth& truth, double dt) {
  ImuMeas m;
  m.t = truth.t;

  // --------------------
  // Gyro = omega_b + bias + noise
  // --------------------
  m.gyro = truth.omega_b + p_.gyro_bias;
  m.gyro(0) += noise_.n(p_.gyro_noise_std);
  m.gyro(1) += noise_.n(p_.gyro_noise_std);
  m.gyro(2) += noise_.n(p_.gyro_noise_std);

  // --------------------
  // Accel: specific force in body
  // f_b = R_bn * (a_ned - g_ned)
  // --------------------
  Vec3 a_ned = Vec3::Zero();
  if (!has_prev_) {
    // First call: can't finite-difference yet
    a_ned.setZero();
    has_prev_ = true;
    v_prev_ned_ = truth.vel;
  } else {
    a_ned = (truth.vel - v_prev_ned_) / dt;
    v_prev_ned_ = truth.vel;
  }

  Vec3 g_ned;
  g_ned << 0.0, 0.0, GRAVITY;  // NED: down positive

  const Mat3 R_nb = R_nb_from_euler(truth.euler); // nav->body

  Vec3 f_b = R_nb * (a_ned - g_ned);

  m.accel = f_b + p_.accel_bias;
  m.accel(0) += noise_.n(p_.accel_noise_std);
  m.accel(1) += noise_.n(p_.accel_noise_std);
  m.accel(2) += noise_.n(p_.accel_noise_std);

  return m;
}

// Euler ZYX (roll=phi, pitch=theta, yaw=psi)
// R_nb maps nav(NED) vectors into body frame.
Mat3 ImuSim::R_nb_from_euler(const Vec3& euler) {
  const double phi = euler(0);
  const double th  = euler(1);
  const double psi = euler(2);

  const double cph = std::cos(phi);
  const double sph = std::sin(phi);
  const double cth = std::cos(th);
  const double sth = std::sin(th);
  const double cps = std::cos(psi);
  const double sps = std::sin(psi);

  Mat3 R;
  // ZYX: R = R_x(phi)*R_y(theta)*R_z(psi) for nav->body
  R(0,0) =  cth*cps;
  R(0,1) =  cth*sps;
  R(0,2) = -sth;

  R(1,0) =  sph*sth*cps - cph*sps;
  R(1,1) =  sph*sth*sps + cph*cps;
  R(1,2) =  sph*cth;

  R(2,0) =  cph*sth*cps + sph*sps;
  R(2,1) =  cph*sth*sps - sph*cps;
  R(2,2) =  cph*cth;

  return R;
}

} // namespace gnc
