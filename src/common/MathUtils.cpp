#include "common/MathUtils.h"

double clamp(double x, double lo, double hi) {
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

double wrapToPi(double angle_rad) {
    double a = std::fmod(angle_rad, 2.0 * PI);

    if (a <= -PI) {
        a += 2.0 * PI;
    }
    if (a > PI) {
        a -= 2.0 * PI;
    }

    return a;
}

Mat<3,3> RotB2N(double phi, double theta, double psi) {
    const double cph = std::cos(phi), sph = std::sin(phi);
    const double cth = std::cos(theta), sth = std::sin(theta);
    const double cps = std::cos(psi), sps = std::sin(psi);

    Mat<3,3> Rz, Ry, Rx;
    Rz << cps, -sps, 0.0,
        sps, cps, 0.0,
        0.0, 0.0, 1.0;

    Ry << cth, 0.0, sth,
        0.0, 1.0, 0.0,
        -sth, 0.0, cth;

    Rx << 1.0, 0.0, 0.0,
        0.0, cph, -sph,
        0.0, sph, cph;

    return Rz * Ry * Rx;
}

Vec<3> eulerRates_ZYX(double phi, double theta, const Vec<3>& omega_b) {

    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);
    const double cth = std::cos(theta);
    const double sth = std::sin(theta);

    double cth_safe = cth;
    if (std::abs(cth_safe) < 1e-9) {
        cth_safe = (cth_safe >= 0.0) ? 1e-9 : -1e-9;
    }

    Mat<3,3> E;
    E(0, 0) = 1.0;
    E(0, 1) = sphi * sth / cth_safe;
    E(0, 2) = cphi * sth / cth_safe;

    E(1, 0) = 0.0;
    E(1, 1) = cphi;
    E(1, 2) = -sphi;

    E(2, 0) = 0.0;
    E(2, 1) = sphi / cth_safe;
    E(2, 2) = cphi / cth_safe;

    Vec<3> euler_dot = E * omega_b;
    return euler_dot;
}

Vec<3> wrapAngles(const Vec<3>& eul) {
    Vec<3> out = eul;
    out(0) = wrapToPi(out(0));
    out(1) = wrapToPi(out(1));
    out(2) = wrapToPi(out(2));
    return out;
}

Mat<3, 3> T_euler(double phi, double th) {
    const double cph = std::cos(phi);
    const double sph = std::sin(phi);
    const double cth = std::cos(th);

    const double tth = std::tan(th);
    const double cth_safe = (std::abs(cth) < 1e-9) ? ((cth >= 0) ? 1e-9 : -1e-9) : cth;

    Mat<3, 3> T;
    T << 1.0, sph* tth, cph* tth,
        0.0, cph, -sph,
        0.0, sph / cth_safe, cph / cth_safe;
    return T;
}

Vec<4> normPWM(const Vec<4>& rawPWM) {
   
    Vec<4> normOut = Vec<4>::Zero();

    normOut = (2.0 * (rawPWM.array() - 1000) / (2000 - 1000) - 1.0).matrix();
   
    const double m = 0.5;
    const double deadzone = 200;
    const double deadmax = deadzone + 1500;
    const double deadmin = 1500 - deadzone; 
    const double k = 1.0 / (deadzone * 2.0);

    normOut(2) = k / m * (softplus(m * (rawPWM(2) - deadmax)) - softplus(m * (deadmin - rawPWM(2))));

    for (int i = 0; i < 4; i++) {
	normOut(i) = clamp(normOut(i),-1,1);
    }

    return normOut;
}

double softplus(double z) {
    if (z > 50.0) return z;
    if (z < -50) return std::exp(z);
    return std::log1p(std::exp(z));
}






