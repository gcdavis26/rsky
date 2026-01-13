#include "control/quad_mixer_x.h"
#include "common/math_util.h"

namespace gnc {

QuadMixerX::QuadMixerX(const MixerParams& p) 
	: p_(p) 
{	
}

Vec4 QuadMixerX::mixToThrusts(const BodyWrench& cmd) const {
  // X config:
  // Each motor contributes to roll/pitch with lever arm L/sqrt(2)
  const double L = p_.arm_length;
  const double a = L / 1.41421356237; // L/sqrt(2)
  const double k = p_.k_yaw;

  // Unknowns: T1 T2 T3 T4
  // Equations (one common convention):
  // Fz = T1 + T2 + T3 + T4
  // Mx (roll)  =  a*( T1 - T2 - T3 + T4 )
  // My (pitch) =  a*( T1 + T2 - T3 - T4 )
  // Mz (yaw)   =  k*( +T1 - T2 + T3 - T4 )  (CCW:+, CW:-)

  // Solve analytically (linear system).
  // Define helpers:
  const double F = cmd.Fz;
  const double Rx = cmd.Mx / a;
  const double Ry = cmd.My / a;
  const double Y  = cmd.Mz / k;

  Vec4 T;
  T(0) = 0.25 * (F + Rx + Ry + Y); // T1
  T(1) = 0.25 * (F - Rx + Ry - Y); // T2
  T(2) = 0.25 * (F - Rx - Ry + Y); // T3
  T(3) = 0.25 * (F + Rx - Ry - Y); // T4

  return T;
}

Vec4 QuadMixerX::clampThrusts(const Vec4& T) const {
  Vec4 Tc = T;
  Tc(0) = clamp(Tc(0), p_.T_min, p_.T_max);
  Tc(1) = clamp(Tc(1), p_.T_min, p_.T_max);
  Tc(2) = clamp(Tc(2), p_.T_min, p_.T_max);
  Tc(3) = clamp(Tc(3), p_.T_min, p_.T_max);
  return Tc;
}

BodyWrench QuadMixerX::thrustsToWrench(const Vec4& T) const {
  const double L = p_.arm_length;
  const double a = L / 1.41421356237;
  const double k = p_.k_yaw;

  BodyWrench w;
  w.Fz = T(0) + T(1) + T(2) + T(3);
  w.Mx = a * ( T(0) - T(1) - T(2) + T(3) );
  w.My = a * ( T(0) + T(1) - T(2) - T(3) );
  w.Mz = k * ( T(0) - T(1) + T(2) - T(3) );

  return w;
}

} // namespace gnc
