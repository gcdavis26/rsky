#pragma once

#include "plant/plant_types.h"
#include "control/mixer_params.h"

namespace gnc {

class QuadMixerX {
public:
  QuadMixerX(const MixerParams& p);

  // Convert desired body wrench into per-motor thrusts [N]
  // Output: motor_thrusts(0..3) = T1..T4
  Vec4 mixToThrusts(const BodyWrench& cmd) const;

  // Clamp thrusts into [T_min, T_max]
  Vec4 clampThrusts(const Vec4& T) const;

  // Convert thrusts [N] back into achieved wrench (useful for logging)
  BodyWrench thrustsToWrench(const Vec4& T) const;

private:
  MixerParams p_;
};

} // namespace gnc
