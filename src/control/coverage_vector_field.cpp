#include "control/coverage_vector_field.h"

#include <cmath>

namespace gnc {

static double vec2Norm(const Vec2& v) {
  return std::sqrt(v(0)*v(0) + v(1)*v(1));
}

Vec2 coverageVectorFieldNE(const Vec3& pos_ned,
                           const Vec3& vel_ned,
                           const SweepParams& p,
                           SweepState& sweepState)
{
  double n  = pos_ned(0);
  double e  = pos_ned(1);

  double vn = vel_ned(0);
  double ve = vel_ned(1);

  Vec2 v_xy;
  v_xy << vn, ve;

  // --- Initialization (MATLAB "isempty") ---
  if (!sweepState.initialized) {
    double width_e = p.e_max - p.e_min;

    int num = (int)std::floor(width_e / p.de_stripe) + 1;
    if (num < 1) num = 1;

    sweepState.numStripes = num;
    sweepState.stripeIdx  = 1;  // start at lowest stripe (e_min)
    sweepState.dir        = 1;  // +north
    sweepState.pass       = 1;  // forward
    sweepState.initialized = true;
  }

  // Clamp stripe index (safety)
  if (sweepState.stripeIdx < 1) sweepState.stripeIdx = 1;
  if (sweepState.stripeIdx > sweepState.numStripes) sweepState.stripeIdx = sweepState.numStripes;

  // Current stripe centerline (along East)
  double e_stripe = p.e_min + (sweepState.stripeIdx - 1) * p.de_stripe;

  // --- 1) Desired velocity from vector field on current stripe ---
  double v_des_n = (double)sweepState.dir * p.v_sweep;
  double v_des_e = -p.kp_cross * (e - e_stripe) - p.kd_cross * ve;

  Vec2 v_des;
  v_des << v_des_n, v_des_e;

  // Normalize to magnitude v_sweep (like MATLAB)
  double mag = vec2Norm(v_des);
  if (mag > 1e-9) {
    v_des = v_des * (p.v_sweep / mag);
  } else {
    // If vector is ~zero, just push along north
    v_des << (double)sweepState.dir * p.v_sweep, 0.0;
  }

  // --- 2) Velocity tracking -> desired acceleration ---
  Vec2 a_des_xy = p.K_vel * (v_des - v_xy);

  // --- 3) North boundary detection ---
  bool at_top    = (n >= (p.n_max - p.n_margin));
  bool at_bottom = (n <= (p.n_min + p.n_margin));

  // --- 4) Pass logic (forward vs reverse) ---
  if (sweepState.pass == 1) {
    // ------- FORWARD: stripes 1 -> numStripes -------
    if ((sweepState.dir > 0) && at_top) {
      if (sweepState.stripeIdx < sweepState.numStripes) {
        sweepState.dir = -1;
        sweepState.stripeIdx = sweepState.stripeIdx + 1;
      } else {
        sweepState.pass = 2;
        sweepState.dir  = -1;
      }
    } else if ((sweepState.dir < 0) && at_bottom) {
      if (sweepState.stripeIdx < sweepState.numStripes) {
        sweepState.dir = +1;
        sweepState.stripeIdx = sweepState.stripeIdx + 1;
      } else {
        sweepState.pass = 2;
        sweepState.dir  = +1;
      }
    }
  } else {
    // ------- REVERSE: stripes numStripes -> 1 -------
    if ((sweepState.dir > 0) && at_top) {
      if (sweepState.stripeIdx > 1) {
        sweepState.dir = -1;
        sweepState.stripeIdx = sweepState.stripeIdx - 1;
      } else {
        // reset to forward pass
        sweepState.pass = 1;
        sweepState.stripeIdx = 1;
        sweepState.dir = 1;
      }
    } else if ((sweepState.dir < 0) && at_bottom) {
      if (sweepState.stripeIdx > 1) {
        sweepState.dir = +1;
        sweepState.stripeIdx = sweepState.stripeIdx - 1;
      } else {
        // also a valid start corner -> reset forward from bottom
        sweepState.pass = 1;
        sweepState.stripeIdx = 1;
        sweepState.dir = 1;
      }
    }
  }

  return a_des_xy;
}

} // namespace gnc
