#pragma once

#include "common/types.h"
#include "plant/quad_params.h"
#include "plant/plant_types.h"

#include "control/sweep_types.h"

namespace gnc {

class OuterLoopCoverage {
public:
  OuterLoopCoverage(const QuadParams& quad,
                    const SweepParams& sweep_params);

  // Computes:
  // - wrench_cmd.Fz
  // - euler_cmd (phi,theta,psi)
  // - a_des_ned for logging/debug
  void compute(const Vec3& pos_ned,
               const Vec3& vel_ned,
               double psi_cmd,
               SweepState& sweepState,
               Vec3& euler_cmd,
               BodyWrench& wrench_cmd,
               Vec3& a_des_ned);

private:
  QuadParams quad_;
  SweepParams p_;
};

} // namespace gnc
