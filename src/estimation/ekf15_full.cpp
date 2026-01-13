#include "estimation/ekf15_full.h"
#include "common/math_util.h"

#include <cmath>

namespace gnc {

Ekf15Full::Ekf15Full(const Params& p, uint32_t /*seed_unused*/)
  : p_(p) {
  reset();

  // Build continuous-time Qc (12x12): [n_g; n_a; n_ba; n_bw]
  Qc_.setZero();
  Qc_.diagonal() <<
    p_.sig_g*p_.sig_g, p_.sig_g*p_.sig_g, p_.sig_g*p_.sig_g,
    p_.sig_acc*p_.sig_acc, p_.sig_acc*p_.sig_acc, p_.sig_acc*p_.sig_acc,
    p_.sig_ba_walk*p_.sig_ba_walk, p_.sig_ba_walk*p_.sig_ba_walk, p_.sig_ba_walk*p_.sig_ba_walk,
    p_.sig_bw_walk*p_.sig_bw_walk, p_.sig_bw_walk*p_.sig_bw_walk, p_.sig_bw_walk*p_.sig_bw_walk;

  Rpos_ = (p_.sig_pos*p_.sig_pos) * Eigen::Matrix3d::Identity();
  Rpsi_ = (p_.sig_psi*p_.sig_psi);
}

void Ekf15Full::reset() {
  x_.setZero();
  P_.setZero();

  have_prev_gyro_ = false;
  gyro_prev_.setZero();
  omega_dot_prev_.setZero();
  predict_count_ = 0;
}

void Ekf15Full::initializeFromOpti(const OptiMeas& opti) {
  // assume initial roll/pitch 0, yaw from opti
  const double phi0 = 0.0;
  const double th0  = 0.0;
  const double psi0 = wrapToPi(opti.psi);

  x_.setZero();
  x_(PHI) = phi0;
  x_(THETA) = th0;
  x_(PSI) = psi0;

  // If opti measures an offset point: p_cg = z_pos - Rnb*r_OPTI
  const Mat3 Rnb0 = RotB2N(phi0, th0, psi0);
  const Vec3 p_cg = opti.pos_ned - Rnb0 * p_.r_OPTI;

  x_(PN) = p_cg(0);
  x_(PE) = p_cg(1);
  x_(PD) = p_cg(2);

  // Keep P_ as-is (tune later)
}

void Ekf15Full::predict(const ImuMeas& imu, double dt) {
  // Estimate omega_dot using the same approach as MATLAB
  Vec3 omega_dot = Vec3::Zero();

  if (!have_prev_gyro_) {
    have_prev_gyro_ = true;
    gyro_prev_ = imu.gyro;
    omega_dot_prev_.setZero();
    omega_dot.setZero();
  } else {
    Vec3 raw = (imu.gyro - gyro_prev_) / dt;
    gyro_prev_ = imu.gyro;

    // low-pass filter
    omega_dot = p_.omega_dot_alpha * omega_dot_prev_ + (1.0 - p_.omega_dot_alpha) * raw;
    omega_dot_prev_ = omega_dot;
  }

  // ----- RK2 (midpoint) state integration (matches MATLAB) -----
  VecX k1 = f_nonlin(x_, imu, omega_dot);
  VecX x_mid = x_ + 0.5 * dt * k1;
  VecX k2 = f_nonlin(x_mid, imu, omega_dot);
  VecX x_pred = x_ + dt * k2;

  // wrap angles
  x_pred.segment<3>(PHI) = wrapAngles(x_pred.segment<3>(PHI));

  // ----- Covariance propagation (continuous Riccati) with RK2 -----
  MatX F = computeF_numeric(x_, imu, omega_dot);
  Eigen::Matrix<double,NX,12> G = computeG(x_);

  MatX Pk = P_;

  MatX Pdot1 = F * Pk + Pk * F.transpose() + G * Qc_ * G.transpose();
  MatX P_mid = Pk + 0.5 * dt * Pdot1;

  // recompute F,G at midpoint for better match (optional but consistent)
  MatX F_mid = computeF_numeric(x_mid, imu, omega_dot);
  Eigen::Matrix<double,NX,12> G_mid = computeG(x_mid);

  MatX Pdot2 = F_mid * P_mid + P_mid * F_mid.transpose() + G_mid * Qc_ * G_mid.transpose();
  MatX P_pred = Pk + dt * Pdot2;

  x_ = x_pred;
  P_ = P_pred;

  predict_count_++;
}

void Ekf15Full::updateOpti(const OptiMeas& opti) {
  const MatX I = MatX::Identity();

  // (a) Position update: z = p_cg + Rnb*r_OPTI
  {
    const Vec3 z = opti.pos_ned;
    const Vec3 h = h_pos(x_);
    Vec3 r = z - h;

    Eigen::Matrix<double,3,NX> H = computeHpos(x_);

    Eigen::Matrix3d S = H * P_ * H.transpose() + Rpos_;
    Eigen::Matrix<double,NX,3> K = P_ * H.transpose() * S.inverse();

    // Joseph form
    x_ = x_ + K * r;
    MatX A = (I - K * H);
    P_ = A * P_ * A.transpose() + K * Rpos_ * K.transpose();

    x_.segment<3>(PHI) = wrapAngles(x_.segment<3>(PHI));
  }

  // (b) Heading update: zpsi = psi + noise
  {
    const double zpsi = opti.psi;
    const double hpsi = h_psi(x_);
    double dpsi = wrapToPi(zpsi - hpsi);

    Eigen::Matrix<double,1,NX> H;
    H.setZero();
    H(0,PSI) = 1.0;

    double S = (H * P_ * H.transpose())(0,0) + Rpsi_;
    Eigen::Matrix<double,NX,1> K = (P_ * H.transpose()) / S;

    x_ = x_ + K * dpsi;

    MatX A = (I - K * H);
    P_ = A * P_ * A.transpose() + K * Rpsi_ * K.transpose();

    x_.segment<3>(PHI) = wrapAngles(x_.segment<3>(PHI));
  }
}

// ------------------- dynamics f(x) -------------------
Ekf15Full::VecX Ekf15Full::f_nonlin(const VecX& x,
                                    const ImuMeas& imu,
                                    const Vec3& omega_dot) const {
  VecX f;
  f.setZero();

  const double phi = x(PHI);
  const double th  = x(THETA);
  const double psi = x(PSI);

  const Vec3 ba = x.segment<3>(BAX);
  const Vec3 bw = x.segment<3>(BP);

  const Vec3 omega_m = imu.gyro;
  const Vec3 acc_m   = imu.accel;

  const Vec3 omega = omega_m - bw;

  // a_corr = acc_m - ba - cross(omega_dot, r_IMU) - cross(omega, cross(omega, r_IMU))
  const Vec3 a_corr =
      (acc_m - ba)
    - omega_dot.cross(p_.r_IMU)
    - omega.cross(omega.cross(p_.r_IMU));

  // Euler rates
  const Mat3 T = T_euler(phi, th);
  const Vec3 eul_dot = T * omega;

  // p_dot = v
  const Vec3 v = x.segment<3>(VN);

  // v_dot = Rnb * a_corr + g_n
  const Mat3 Rnb = RotB2N(phi, th, psi); // body->NED
  Vec3 g_n; g_n << 0.0, 0.0, p_.g;

  const Vec3 v_dot = Rnb * a_corr + g_n;

  f.segment<3>(PHI) = eul_dot;
  f.segment<3>(PN)  = v;
  f.segment<3>(VN)  = v_dot;

  // bias walks are zero-mean random walks (modeled via process noise), so f=0
  // f(BAX..BR) stays 0
  return f;
}

// ------------------- F numeric (central diff) -------------------
Ekf15Full::MatX Ekf15Full::computeF_numeric(const VecX& x,
                                            const ImuMeas& imu,
                                            const Vec3& omega_dot) const {
  MatX F;
  F.setZero();

  const double eps = p_.eps_F;

  for (int j = 0; j < NX; j++) {
    VecX dx = VecX::Zero();
    dx(j) = eps;

    VecX f1 = f_nonlin(x + dx, imu, omega_dot);
    VecX f0 = f_nonlin(x - dx, imu, omega_dot);

    F.col(j) = (f1 - f0) / (2.0 * eps);
  }

  return F;
}

// ------------------- G (matches MATLAB) -------------------
Eigen::Matrix<double, Ekf15Full::NX, 12> Ekf15Full::computeG(const VecX& x) const {
  Eigen::Matrix<double,NX,12> G;
  G.setZero();

  const double phi = x(PHI);
  const double th  = x(THETA);
  const double psi = x(PSI);

  const Mat3 T = T_euler(phi, th);
  const Mat3 Rnb = RotB2N(phi, th, psi);

  // gyro noise -> Euler rates through T
  G.block<3,3>(PHI, 0) = T;

  // accel noise -> velocity through Rnb
  G.block<3,3>(VN, 3) = Rnb;

  // bias random walks
  G.block<3,3>(BAX, 6) = Mat3::Identity();
  G.block<3,3>(BP,  9) = Mat3::Identity();

  return G;
}

// ------------------- measurement model -------------------
Vec3 Ekf15Full::h_pos(const VecX& x) const {
  const double phi = x(PHI);
  const double th  = x(THETA);
  const double psi = x(PSI);

  const Mat3 Rnb = RotB2N(phi, th, psi);

  Vec3 p_cg;
  p_cg << x(PN), x(PE), x(PD);

  return p_cg + Rnb * p_.r_OPTI;
}

double Ekf15Full::h_psi(const VecX& x) const {
  return x(PSI);
}

Eigen::Matrix<double,3,Ekf15Full::NX> Ekf15Full::computeHpos(const VecX& x) const {
  Eigen::Matrix<double,3,NX> H;
  H.setZero();

  const double eps = p_.eps_H; // or a dedicated eps_H

  for (int j = 0; j < NX; j++) {
    VecX dx = VecX::Zero();
    dx(j) = eps;

    Vec3 h1 = h_pos(x + dx);
    Vec3 h0 = h_pos(x - dx);

    H.col(j) = (h1 - h0) / (2.0 * eps);
  }

  return H;
}

} // namespace gnc
