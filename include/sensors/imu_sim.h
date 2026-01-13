#pragma once

#include <cstdint>

#include "common/types.h"
#include "plant/quad_params.h"
#include "plant/quad_state.h"   // make sure this is your truth type header
#include "sensors/sensor_types.h"
#include "sensors/noise_gaussian.h"

namespace gnc {

struct ImuSimParams {
  // White noise standard deviations
  double gyro_noise_std = 0.01*PI/180*sqrt(100);   // rad/s
  double accel_noise_std = 0.029;   // m/s^2

  // Optional constant biases (set to 0 if you don't want them yet)
  Vec3 gyro_bias = Vec3::Zero();   // rad/s
  Vec3 accel_bias = Vec3::Zero();  // m/s^2
};

class ImuSim {
public:
  ImuSim(const QuadParams& quad, const ImuSimParams& p, uint32_t seed = 1);

  void reset();

  // dt is needed to estimate acceleration from velocity
  ImuMeas step(const QuadStateTruth& truth, double dt);

private:
  QuadParams quad_;
  ImuSimParams p_;
  GaussianNoise noise_;

  bool has_prev_;
  Vec3 v_prev_ned_;

private:
  static Mat3 R_nb_from_euler(const Vec3& euler);
};

} // namespace gnc
