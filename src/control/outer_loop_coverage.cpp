#include "control/outer_loop_coverage.h"

#include <cmath>

#include "common/math_util.h"
#include "control/coverage_vector_field.h"

namespace gnc {

OuterLoopCoverage::OuterLoopCoverage(const QuadParams& quad,
                                     const SweepParams& sweep_params)
  : quad_(quad), p_(sweep_params)
{
}

void OuterLoopCoverage::compute(const Vec3& pos_ned,
                                const Vec3& vel_ned,
                                double psi_cmd,
                                SweepState& sweepState,
                                Vec3& euler_cmd,
                                BodyWrench& wrench_cmd,
                                Vec3& a_des_ned)
{
  double g = GRAVITY;

  // --- Horizontal accel from vector field (returns updated sweepState) ---
  Vec2 a_des_xy = coverageVectorFieldNE(pos_ned, vel_ned, p_, sweepState);

  // --- Vertical: hold survey altitude (NED down) ---
  double d   = pos_ned(2);
  double vd  = vel_ned(2);
  double d_ref = p_.d_survey;

  double e_d  = d_ref - d;
  double e_vd = 0.0 - vd;

  double a_des_d = p_.Kp_d * e_d + p_.Kd_d * e_vd;

  if (a_des_d >  p_.a_d_max) a_des_d =  p_.a_d_max;
  if (a_des_d < -p_.a_d_max) a_des_d = -p_.a_d_max;

  // Combine desired accel (NED)
  a_des_ned(0) = a_des_xy(0); // a_n
  a_des_ned(1) = a_des_xy(1); // a_e
  a_des_ned(2) = a_des_d;     // a_d

  // --- Convert desired horizontal accel into yaw-compensated components ---
  double c = std::cos(psi_cmd);
  double s = std::sin(psi_cmd);

  double a_n = a_des_ned(0);
  double a_e = a_des_ned(1);

  // Rotate inertial accel into body-horizontal axes (same form as MATLAB)
  double aBx =  c * a_n + s * a_e;
  double aBy = -s * a_n + c * a_e;

  // --- Small-angle mapping (NED) ---
  // theta_cmd ≈ -aBx / g
  // phi_cmd   ≈  aBy / g
  double phi_cmd   =  aBy / g;
  double theta_cmd = -aBx / g;

  // Clamp angles
  phi_cmd   = clamp(phi_cmd,   -p_.max_angle, p_.max_angle);
  theta_cmd = clamp(theta_cmd, -p_.max_angle, p_.max_angle);

  // Denominator tilt correction (like MATLAB)
  double den = std::cos(phi_cmd) * std::cos(theta_cmd);
  if (den < 0.2) den = 0.2;

  // --- Thrust command (NED) ---
  // NED vertical dynamics: a_d = g - Fz/m  (for small angles, with our thrust definition)
  // => Fz = m * (g - a_d) / den
  double Fz_cmd = quad_.m * (g - a_des_ned(2)) / den;

  // Clamp thrust
  Fz_cmd = clamp(Fz_cmd, p_.Fz_min, p_.Fz_max);

  // Outputs
  euler_cmd(0) = phi_cmd;
  euler_cmd(1) = theta_cmd;
  euler_cmd(2) = psi_cmd;

  wrench_cmd.Fz = Fz_cmd;
}

} // namespace gnc
