#include "control/InnerLoop.h"

Vec<3> InnerLoop::computeWrench(
    const Vec<3>& att_cmd,
    double yaw_rate_cmd,
    const Vec<3>& att,
    const Vec<3>& omega,
    double dt)
{
    // ---- OUTER LOOP: attitude → desired rate ----
    Vec<3> attErr = wrapAngles(att_cmd - att);

    // Leaky integrator on attitude error
    Vec<3> attIntInput;
    attIntInput.segment<2>(0) = attErr.segment<2>(0);
    attIntInput(2) = (yaw_rate_cmd == 0.0) ? attErr(2) : 0.0;

    x4_att += (attIntInput - x4_att / tauI_att) * dt;

    Vec<3> desired_rate = kp_att.cwiseProduct(attErr)
        + ki_att.cwiseProduct(x4_att);

    // Yaw rate mode: bypass outer loop entirely
    if (yaw_rate_cmd != 0.0) {
        desired_rate(2) = yaw_rate_cmd;
    }

    // ---- INNER LOOP: rate → torque ----
    Vec<3> rateErr = desired_rate - omega;

    // Leaky integrator on rate error
    x4_rate += (rateErr - x4_rate / tauI_rate) * dt;

    Vec<3> u = kp_rate.cwiseProduct(rateErr)
        + ki_rate.cwiseProduct(x4_rate);

    // ---- Actuator lowpass (unchanged) ----
    x5 += ((u - x5) / tauA) * dt;
    Vec<3> wrench = x5;

    // ---- Saturation (unchanged) ----
    wrench(0) = clamp(wrench(0), -Mx_max, Mx_max);
    wrench(1) = clamp(wrench(1), -My_max, My_max);
    wrench(2) = clamp(wrench(2), -Mz_max, Mz_max);
    return wrench;
}