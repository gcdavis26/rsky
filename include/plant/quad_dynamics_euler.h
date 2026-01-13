#pragma once

#include <random>

#include "common/types.h"
#include "plant/quad_params.h"
#include "plant/quad_state.h"

namespace gnc {

// 6-DOF quad dynamics with Euler angles, using NED inertial frame.
class QuadDynamics {
public:
  QuadDynamics(const QuadParams& params);

  // bodyWrench = [Fz, Mx, My, Mz]
  // Fz is positive thrust magnitude [N]
  void step(QuadStateTruth& state, double dt, const Vec4& bodyWrench);

private:
  QuadParams params_;

  // For turbulence a = a + sigma_turb .* randn(3,1)
  std::mt19937 rng_;
  std::normal_distribution<double> norm_;
};

} // namespace gnc
