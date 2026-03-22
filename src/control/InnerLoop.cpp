#include "control/InnerLoop.h"

Vec<3> InnerLoop::computeWrench(
    const Vec<3>& att_cmd,
    double yaw_rate_cmd,
    const Vec<3>& att,
    const Vec<3>& omega,
    double dt)
{
    Vec<3> attErr = wrapAngles(att_cmd - att);

    Vec<3> intErr = Vec<3>::Zero();
    intErr(0) = attErr(0);
    intErr(1) = attErr(1);

     if (yaw_rate_cmd == 0.0) intErr(2) = attErr(2);

    Vec<3> x4_dot = intErr - x4 / tauI;
    x4 += x4_dot * dt;

    // clamp integrator state to prevent windup
    //x4(0) = clamp(x4(0), -x4max_x, x4max_x);

    // 3) Raw (unfiltered) wrench command u
    Vec<3> u = kp.cwiseProduct(attErr)
        + ki.cwiseProduct(x4)
        - kd.cwiseProduct(omega);

    // Keep your yaw-rate override
    if (yaw_rate_cmd != 0.0) {
        // Optionally: disable yaw attitude P when in yaw-rate mode
         u(2) = 0.0 + ki(2)*x4(2) - kd(2)*omega(2);

        u(2) = kd(2) * (yaw_rate_cmd - omega(2));
        // If you want integral on yaw rate:
         double yawRateErr = yaw_rate_cmd - omega(2);
         x4(2) += (yawRateErr - x4(2)/tauI) * dt;
         u(2) += ki(2) * x4(2);
    }

    // 4) Actuator low-pass filter (x5): x5_dot = (u - x5)/tauA
    Vec<3> x5_dot = (u - x5) / tauA;
    x5 += x5_dot * dt;

    Vec<3> wrench = x5;

    // 5) Saturation (keep yours)
    wrench(0) = clamp(wrench(0), -Mx_max, Mx_max);
    wrench(1) = clamp(wrench(1), -My_max, My_max);
    wrench(2) = clamp(wrench(2), -Mz_max, Mz_max);

    return wrench;
}