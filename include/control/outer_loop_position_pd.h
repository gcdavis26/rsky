#pragma once

#include "common/types.h"
#include "plant/quad_params.h"
#include "plant/plant_types.h"
#include "control/position_pd_params.h"

namespace gnc {

class OuterLoopPositionPD {
public:
  OuterLoopPositionPD(const PositionPDParams& p,
                      const QuadParams& quad);

  // Inputs:
  //   pos_cmd_ned : desired [n,e,d]
  //   pos_ned     : measured position
  //   vel_ned     : measured velocity
  //
  // Outputs:
  //   euler_cmd : [phi_cmd, theta_cmd, psi_cmd]
  //   wrench_cmd.Fz updated
  void compute(const Vec3& pos_cmd_ned,
               const Vec3& pos_ned,
               const Vec3& vel_ned,
			   const Vec3& euler,
               Vec3& euler_cmd,
               BodyWrench& wrench_cmd);

private:
  PositionPDParams p_;
  QuadParams quad_;
};

} // namespace gnc
