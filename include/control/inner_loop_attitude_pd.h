#pragma once

#include "common/types.h"
#include "plant/plant_types.h"
#include "control/attitude_pd_params.h"

namespace gnc {

class InnerLoopAttitudePD {
public:
  InnerLoopAttitudePD(const AttitudePDParams& p);

  // Compute body moments to track desired Euler angles.
  // Returns a wrench with only Mx/My/Mz filled (Fz should be set elsewhere).
  BodyWrench compute(const Vec3& euler_cmd,
                     const Vec3& euler_meas,
                     const Vec3& omega_meas);

private:
  AttitudePDParams p_;
};

} // namespace gnc
