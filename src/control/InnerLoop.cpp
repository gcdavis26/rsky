#include "control/InnerLoop.h"

Vec<3> InnerLoop::computeWrench(
    const Vec<3>& att_cmd,
    double yaw_rate_cmd,
    const Vec<3>& att,
    const Vec<3>& omega,
    double dt,
    bool armCheck)
{
    Armed = armCheck;

    // ---- OUTER LOOP: attitude → desired rate ----
    Vec<3> attErr = wrapAngles(att_cmd - att);

    // True integrator on attitude error (Standard integration)
    Vec<3> attIntInput;
    if (Armed) {
      attIntInput.segment<2>(0) = attErr.segment<2>(0);
      attIntInput(2) = (yaw_rate_cmd == 0.0) ? attErr(2) : 0.0;
    }
    else {
	attIntInput = Vec<3>::Zero();
    }
    x4_att += attIntInput * dt;
    
    // Standard anti-windup clamp for the outer loop
    x4_att(0) = clamp(x4_att(0), -5.0, 5.0);
    x4_att(1) = clamp(x4_att(1), -5.0, 5.0);
    x4_att(2) = clamp(x4_att(2), -5.0, 5.0);

    Vec<3> desired_rate = kp_att.cwiseProduct(attErr)
        + ki_att.cwiseProduct(x4_att);

    // Yaw rate mode: bypass outer loop entirely
    if (std::abs(yaw_rate_cmd) >  0.0001) {
        yawLatch = false;
        desired_rate(2) = yaw_rate_cmd;
    }

   if(yawLatch){
       desired_rate(2) = omega(2);
   }

    Vec<3> rateErr = desired_rate - omega;

    // Pure Integrator with Back-Calculation
    x4_rate += rateErr * dt;

    // Calculate the theoretical unclamped PID output
    Vec<3> u_unclamped = kp_rate.cwiseProduct(rateErr) + ki_rate.cwiseProduct(x4_rate);

    // Clamp the output to your known torque limits
    Vec<3> u_clamped;
    u_clamped(0) = clamp(u_unclamped(0), -Mx_max, Mx_max);
    u_clamped(1) = clamp(u_unclamped(1), -My_max, My_max);
    u_clamped(2) = clamp(u_unclamped(2), -Mz_max, Mz_max);

    // Back-calculation "Smart Leak"
    // If u_unclamped is within bounds, excess is 0 (acts as pure integrator).
    // If u_unclamped exceeds bounds, it instantly bleeds the excess off the integrator.
    double back_calc_gain = 2.0; // You can tune this to dictate how aggressively the leak reacts
    Vec<3> excess = u_unclamped - u_clamped;
    x4_rate -= back_calc_gain * excess * dt; 

    // ---- Actuator lowpass ----
    // Feed the safe, clamped command into the actuator low-pass filter
    x5 += ((u_clamped - x5) / tauA) * dt;
    Vec<3> wrench = x5;
    

    // Final safety clamp before outputting
    wrench(0) = clamp(wrench(0), -Mx_max, Mx_max);
    wrench(1) = clamp(wrench(1), -My_max, My_max);
    wrench(2) = clamp(wrench(2), -Mz_max, Mz_max);
    return wrench;
}
