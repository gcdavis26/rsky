#pragma once

#include "common/types.h"

namespace gnc {

// Parameters needed by the MATLAB dynamics (converted to NED usage)
struct QuadParams {
  double m;     // mass [kg]

  double Ix;    // [kg*m^2]
  double Iy;    // [kg*m^2]
  double Iz;    // [kg*m^2]

  Vec3 sigma_turb; // [m/s^2] per-axis accel noise (NED axes)

  QuadParams() {
    m = MASS;
    Ix = 0.01;
    Iy = 0.01;
    Iz = 0.02;

    sigma_turb.setZero(); // set nonzero if you want turbulence
  }
};

} // namespace gnc
