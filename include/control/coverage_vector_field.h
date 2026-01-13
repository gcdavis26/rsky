#pragma once

#include "common/types.h"
#include "control/sweep_types.h"

namespace gnc {

// Returns desired horizontal acceleration [a_n; a_e] and updates sweepState.
Vec2 coverageVectorFieldNE(const Vec3& pos_ned,
                           const Vec3& vel_ned,
                           const SweepParams& p,
                           SweepState& sweepState);

} // namespace gnc
