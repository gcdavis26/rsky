#pragma once

#include <cmath>
#include "common/types.h"

namespace gnc {

double clamp(double x, double lo, double hi);

double wrapToPi(double angle_rad);

Mat3 RotB2N(double phi, double theta, double psi);

Vec3 eulerRates_ZYX(double phi, double theta, const Vec3& omega_b);

Vec3 wrapAngles(const Vec3& eul);

Mat3 T_euler(double phi, double th);   

} // namespace gnc
