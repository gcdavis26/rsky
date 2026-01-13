#pragma once

namespace gnc {

struct PositionPDParams {
  // Position gains
  double kp_n;
  double kp_e;
  double kp_d;

  // Velocity damping
  double kd_n;
  double kd_e;
  double kd_d;

  // Limits
  double phi_max;    // [rad]
  double theta_max;  // [rad]
  double Fz_min;     // [N]
  double Fz_max;     // [N]

  PositionPDParams() {
    kp_n = 1.0;
    kp_e = 1.0;
    kp_d = 2.0;

    kd_n = 1.2;
    kd_e = 1.2;
    kd_d = 3.0;

    phi_max   = 20.0 * 3.141592653589793 / 180.0;
    theta_max = 20.0 * 3.141592653589793 / 180.0;

    Fz_min = 0.0;
    Fz_max = 2.0 * GRAVITY;
  }
};

} // namespace gnc
