#pragma once

namespace gnc {

struct SweepState {
  int stripeIdx;
  int dir;         // +1 or -1 (direction along North)
  int numStripes;
  int pass;        // 1 = forward, 2 = reverse
  bool initialized;

  SweepState() {
    stripeIdx = 1;
    dir = 1;
    numStripes = 1;
    pass = 1;
    initialized = false;
  }
};

struct SweepParams {
  // Area bounds (NED horizontal)
  double buffer;
  double n_min;
  double n_max;
  double e_min;
  double e_max;

  // Stripe spacing along East
  double de_stripe;

  // End detection margin on north boundaries
  double n_margin;

  // Vector field settings
  double v_sweep;    // desired speed magnitude [m/s]
  double kp_cross;    // pull-to-stripe gain
  double kd_cross;
  double K_vel;      // velocity tracking gain (accel command)

  // Vertical hold (NED down)
  double d_survey;   // desired down position [m] (constant)
  double Kp_d;
  double Kd_d;
  double a_d_max;    // max vertical accel command magnitude [m/s^2]

  // Command limits
  double max_angle;  // [rad]
  double Fz_min;     // [N]
  double Fz_max;     // [N]

  SweepParams() {
	buffer = 0.25;
    n_min = 0 + buffer;
    n_max =  9.144 - buffer;
    e_min = 0 + buffer;
    e_max =  4.572 - buffer;
  
    de_stripe = 2.0;
    n_margin  = 0.50;

    v_sweep  = 1.0;
    kp_cross  = 1.0;
	kd_cross = 0.5;
    K_vel    = 1.0;

    d_survey = -2.0;
    Kp_d = 2.0;
    Kd_d = 2.0;
    a_d_max = 2.0;

    max_angle = 10 * PI / 180.0;
    Fz_min = 0.0;
    Fz_max = 2 * MASS * GRAVITY;
  }
};

}