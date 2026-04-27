#pragma once
#include "common/MathUtils.h"
#include "common/LowPass.h"

class InnerLoop {
public:
    Vec<3> computeWrench(
        const Vec<3>& att_cmd,
        double yaw_rate_cmd,
        const Vec<3>& att,
        const Vec<3>& omega,
        double dt);

private:
    bool yawLatch = true;
    // ---- Outer loop (attitude → desired rate) ----
    static const inline Vec<3> kp_att{ 8.0, 8.0, 4.0 };
    static const inline Vec<3> ki_att{ 0.000, 0.000, 0.00};
    double tauI_att = 0.025;
    Vec<3> x4_att = Vec<3>::Zero();

    // ---- Inner loop (rate → torque) ----
    static const inline Vec<3> kp_rate{ 0.050, 0.050, 0.075 };
    static const inline Vec<3> ki_rate{ 0.000, 0.000, 0.0};

    double tauI_rate = 0.025;
    Vec<3> x4_rate = Vec<3>::Zero();

    // ---- Actuator filter ----
    double tauA = 0.015;
    Vec<3> x5 = Vec<3>::Zero();

    // ---- Saturation limits ----
    static constexpr double Mx_max = 1.0;
    static constexpr double My_max = 1.0;
    static constexpr double Mz_max = 0.5;
};
