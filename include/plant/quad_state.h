#pragma once

#include "common/types.h"

namespace gnc {

// Truth state using NED (North-East-Down)
struct QuadStateTruth {
  double t;        // [s]

  Vec3 pos;      // [m]  (n, e, d)
  Vec3 vel;      // [m/s] (vn, ve, vd)

  Vec3 euler;      // [rad] (phi, theta, psi) roll, pitch, yaw
  Vec3 omega_b;    // [rad/s] (p, q, r) body rates

  QuadStateTruth() {
    t = 0.0;
    pos.setZero();
    vel.setZero();
    euler.setZero();
    omega_b.setZero();
  }
};

} // namespace gnc
