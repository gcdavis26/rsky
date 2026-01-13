#include "control/inner_loop_attitude_pd.h"

#include "common/math_util.h"

namespace gnc {

InnerLoopAttitudePD::InnerLoopAttitudePD(const AttitudePDParams& p)
  : p_(p)
{
}

BodyWrench InnerLoopAttitudePD::compute(const Vec3& euler_cmd,
                                       const Vec3& euler_meas,
                                       const Vec3& omega_meas)
{
  // Errors in Euler angles
  double e_phi   = wrapToPi(euler_cmd(0) - euler_meas(0));
  double e_theta = wrapToPi(euler_cmd(1) - euler_meas(1));
  double e_psi   = wrapToPi(euler_cmd(2) - euler_meas(2));

  // PD on angles + body rates (rate damping)
  double Mx = p_.kp_roll  * e_phi   - p_.kd_p * omega_meas(0);
  double My = p_.kp_pitch * e_theta - p_.kd_q * omega_meas(1);
  double Mz = p_.kp_yaw   * e_psi   - p_.kd_r * omega_meas(2);

  // Clamp moments
  Mx = clamp(Mx, -p_.Mx_max, p_.Mx_max);
  My = clamp(My, -p_.My_max, p_.My_max);
  Mz = clamp(Mz, -p_.Mz_max, p_.Mz_max);

  BodyWrench w;

  w.Mx = Mx;
  w.My = My;
  w.Mz = Mz;
  return w;
}

} // namespace gnc
