#pragma once

#include <Eigen/Dense>
#include <cstdint>

#include "common/types.h"
#include "sensors/sensor_types.h"

namespace gnc {

class Ekf15Full {
public:
  static constexpr int NX = 15;

  using VecX  = Eigen::Matrix<double, NX, 1>;
  using MatX  = Eigen::Matrix<double, NX, NX>;
  using Mat12 = Eigen::Matrix<double, 12, 12>;

  // State indices (match MATLAB exactly)
  enum Idx : int {
    PHI=0, THETA=1, PSI=2,
    PN=3,  PE=4,    PD=5,
    VN=6,  VE=7,    VD=8,
    BAX=9, BAY=10,  BAZ=11,
    BP=12, BQ=13,   BR=14
  };

  struct Params {
    double g = GRAVITY;     // NED down positive gravity magnitude

    // Lever arms in BODY frame [m]
    Vec3 r_IMU  = Vec3::Zero();
    Vec3 r_OPTI = Vec3::Zero(); // OptiTrack measures CG by default; set if not

    // Process noise (continuous-time spectral densities)
    double sig_g   = 0.01*PI/180; // rad/s/sqrt(Hz) 
    double sig_acc = 2.94e-3; // m/s^2/sqrt(Hz)  
    double sig_ba_walk = 0.0; // (m/s^2)/sqrt(s)
    double sig_bw_walk = 0.0; // (rad/s)/sqrt(s)

    // Measurement noise
    double sig_pos = 5e-5; // m
    double sig_psi = 0.00087; // rad

    // Numerical Jacobian epsilons
    double eps_F = 1e-6;
    double eps_H = 1e-7;

    // Omega-dot low-pass (matches MATLAB)
    double omega_dot_alpha = 0.7;
  };

  Ekf15Full(const Params& p = Params(), uint32_t seed_unused = 0);

  void reset();

  // Initialize like MATLAB: if you have an opti measurement at start,
  // you can set p from z_pos - Rnb*r_OPTI and yaw from z_psi.
  void initializeFromOpti(const OptiMeas& opti);

  // Predict at IMU rate
  void predict(const ImuMeas& imu, double dt);

  // Update at OptiTrack rate (pos + yaw)
  void updateOpti(const OptiMeas& opti);

  const VecX& x() const { return x_; }
  const MatX& P() const { return P_; }

private:
  Params p_;

  VecX x_;
  MatX P_;

  Mat12 Qc_;                           // continuous process noise (12x12)
  Eigen::Matrix<double,3,3> Rpos_;     // position measurement cov
  double Rpsi_;                        // yaw measurement variance

  // Omega-dot estimator state
  bool have_prev_gyro_;
  Vec3 gyro_prev_;
  Vec3 omega_dot_prev_;
  int predict_count_;

private:
  // Core continuous dynamics f(x) (uses current imu measurement held constant)
  VecX f_nonlin(const VecX& x, const ImuMeas& imu, const Vec3& omega_dot) const;

  // Numerical Jacobians
  MatX computeF_numeric(const VecX& x, const ImuMeas& imu, const Vec3& omega_dot) const;

  // Noise input matrix G (15x12) for [n_g; n_a; n_ba; n_bw]
  Eigen::Matrix<double,NX,12> computeG(const VecX& x) const;

  // Measurement model: z_pos = p_cg + Rnb*r_OPTI, z_psi = psi
  Vec3 h_pos(const VecX& x) const;
  double h_psi(const VecX& x) const;

  Eigen::Matrix<double,3,NX> computeHpos(const VecX& x) const;

};

} // namespace gnc
