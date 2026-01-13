#pragma once

#include "common/types.h"

namespace gnc {

// -------------------- IMU measurement --------------------
// gyro: body rates [rad/s]
// accel: specific force in body [m/s^2]
struct ImuMeas {
  double t = 0.0;
  Vec3 gyro = Vec3::Zero();
  Vec3 accel = Vec3::Zero();
};

// -------------------- OptiTrack measurement --------------------
// pos: NED position [m]
// psi: yaw [rad]
struct OptiMeas {
  double t = 0.0;
  Vec3 pos_ned = Vec3::Zero();
  double psi = 0.0;
};

} // namespace gnc
