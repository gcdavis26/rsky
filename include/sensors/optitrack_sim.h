#pragma once

#include <cstdint>

#include "common/types.h"
#include "plant/quad_state.h"
#include "sensors/sensor_types.h"
#include "sensors/noise_gaussian.h"

namespace gnc {

struct OptiSimParams {
  double pos_noise_std = 0.0005; // m
  double psi_noise_std = 0.0087; // rad
};

class OptiTrackSim {
public:
  explicit OptiTrackSim(const OptiSimParams& p, uint32_t seed = 2);

  OptiMeas step(const QuadStateTruth& truth);

private:
  OptiSimParams p_;
  GaussianNoise noise_;

private:
  static double wrapToPi(double a);
};

} // namespace gnc
