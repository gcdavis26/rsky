#pragma once

namespace gnc {

struct AttitudePDParams {
  // Angle gains (rad -> N*m)
  double kp_roll;
  double kp_pitch;
  double kp_yaw;

  // Rate gains ((rad/s) -> N*m)
  double kd_p;
  double kd_q;
  double kd_r;

  // Moment limits [N*m] to avoid insane commands
  double Mx_max;
  double My_max;
  double Mz_max;

  AttitudePDParams() {
    kp_roll  = 0.5;
    kp_pitch = 0.5;
    kp_yaw   = 0.75;

    kd_p = 0.1;
    kd_q = 0.1;
    kd_r = 0.1;

    Mx_max = 1.0;
    My_max = 1.0;
    Mz_max = 0.5;
  }
};

} // namespace gnc
