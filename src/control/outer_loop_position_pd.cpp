#include "control/outer_loop_position_pd.h"

#include "common/math_util.h"

namespace gnc {

OuterLoopPositionPD::OuterLoopPositionPD(const PositionPDParams& p,
                                         const QuadParams& quad)
  : p_(p), quad_(quad)
{
}

void OuterLoopPositionPD::compute(const Vec3& pos_cmd,
                                  const Vec3& pos,
                                  const Vec3& vel,
								  const Vec3& euler,
                                  Vec3& euler_cmd,
                                  BodyWrench& wrench_cmd)
{
  // Position errors (NED)
  double e_n = pos_cmd(0) - pos(0);
  double e_e = pos_cmd(1) - pos(1);
  double e_d = pos_cmd(2) - pos(2);

  // Velocity errors (desired velocity = 0)
  double ev_n = -vel(0);
  double ev_e = -vel(1);
  double ev_d = -vel(2);

  // Desired accelerations
  double a_n = p_.kp_n * e_n + p_.kd_n * ev_n;
  double a_e = p_.kp_e * e_e + p_.kd_e * ev_e;
  double a_d = p_.kp_d * e_d + p_.kd_d * ev_d;

  // --- Convert desired horizontal accel to roll/pitch ---
  // Small-angle approximation

  double ax_B = cos(euler(2))*a_n + sin(euler(2))*a_e;
  double ay_B = -sin(euler(2))*a_n + cos(euler(2))*a_e;


  double phi_cmd   =  ay_B / GRAVITY;
  double theta_cmd = -ax_B / GRAVITY;

  // Clamp tilt
  phi_cmd   = clamp(phi_cmd,   -p_.phi_max,   p_.phi_max);
  theta_cmd = clamp(theta_cmd, -p_.theta_max, p_.theta_max);

  // --- Vertical thrust command ---
  // a_d = g - Fz/m  (NED, positive down)
  // => Fz = m * (g - a_d)
  
  
  double den = cos(phi_cmd)*cos(theta_cmd);
  den = clamp(den,0.2,den);
  double Fz_cmd = quad_.m * (GRAVITY - a_d) / den; ;

  Fz_cmd = clamp(Fz_cmd, p_.Fz_min, p_.Fz_max);

  // Outputs
  euler_cmd(0) = phi_cmd;
  euler_cmd(1) = theta_cmd;
  euler_cmd(2) = 0.0; // yaw hold for now

  wrench_cmd.Fz = Fz_cmd;
}

} // namespace gnc
