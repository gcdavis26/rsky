// EKF.h
#pragma once

#include "common/MathUtils.h"
#include "sensors/ImuSim.h"

#ifdef PLATFORM_LINUX
    #include "sensors/IMUHandler.h"
    #include "mocap/mocapHandler.h"
#endif

#include "sensors/OptiSim.h"

class EKF {
public:
    static constexpr int NX = 15;

    // state index map (15 states)
    enum Idx : int {
        PHI = 0, THETA = 1, PSI = 2,
        PN = 3, PE = 4, PD = 5,
        VN = 6, VE = 7, VD = 8,
        BAX = 9, BAY = 10, BAZ = 11,
        BP = 12, BQ = 13, BR = 14
    };

    EKF();

    EKF(const Vec<6>& bias);

    bool init = false;

    template<typename OptiT>
    void initializeFromOpti(const OptiT& opti);

    void predict(const Vec<6>& imu, double dt);

    template<typename OptiT>
    void correct(const OptiT& opti);

    Vec<NX> getx() const { return x_est; }
    Mat<NX, NX> getP() const { return P; }
    double getHealth() const { return nisAvg; }
    Vec<4> getRes() const { return res; }

private:
    // internal canonical measurement types for EKF math
    struct ImuMeas {
        Vec<3> gyro = Vec<3>::Zero();
        Vec<3> accel = Vec<3>::Zero();
    };

    struct OptiMeas {
        Vec<3> pos = Vec<3>::Zero();
        double psi = 0.0;
    };

    // state + covariance
    Vec<NX> x_est = Vec<NX>::Zero();
    Mat<NX, NX> P = Mat<NX, NX>::Zero();
    Mat<4, 4> S = Mat<4, 4>::Zero();
    Vec<4> res = Vec<4>::Zero();

    // geometry
    Vec<3> r_IMU{ -0.013,-0.009,-0.058 }; // (-1.3cm,-0.9cm,-5.8cm)
    Vec<3> r_OPTI = Vec<3>::Zero();

    // process noise (continuous-time)
    double sig_g = 0.002;
    double sig_acc = 0.015;
    double sig_ba_walk = 1e-4;
    double sig_bw_walk = 1e-6;

    // measurement noise
    double sig_pos = 0.005;
    double sig_psi = 0.005;

    // numerics
    double eps_F = 1e-6;
    double eps_H = 1e-6;
    double omega_dot_alpha = 0.7;

    // omega_dot estimation
    bool have_prev_gyro = false;
    Vec<3> gyro_prev = Vec<3>::Zero();
    Vec<3> omega_dot_prev = Vec<3>::Zero();

    // cached noise covariances
    Mat<12, 12> Qc = Mat<12, 12>::Zero();   // [ng(3) na(3) nba(3) nbw(3)]
    Mat<3, 3> Rpos = Mat<3, 3>::Zero();
    double Rpsi = 0.0;

    // non-templated EKF core functions
    void initializeFromOptiImpl(const OptiMeas& opti);
    void predictImpl(const ImuMeas& imu, double dt);
    void correctImpl(const OptiMeas& opti);

    // dynamics + Jacobians
    Vec<NX> f_nonlin(const Vec<NX>& x, const ImuMeas& imu, const Vec<3>& omega_dot) const;
    Mat<NX, NX> computeF(const Vec<NX>& x, const ImuMeas& imu, const Vec<3>& omega_dot, const Vec<NX>& k1) const;
    Mat<NX, 12> computeG(const Vec<NX>& x) const;

    // measurement model + Jacobian
    Vec<3> h_pos(const Vec<NX>& x) const;
    double h_psi(const Vec<NX>& x) const;

    // health monitoring
    double nisAvg = 0.0;
    bool nisInit = false;

    // helpers (from your MathUtils)
    // Mat<3,3> RotB2N(double phi, double th, double psi);
    // Mat<3,3> T_euler(double phi, double th);
    // double   wrapToPi(double a);
    // Vec<3>   wrapAngles(const Vec<3>& rpy);
};


// ===================== Template definitions =====================

template<typename OptiT>
void EKF::initializeFromOpti(const OptiT& opti)
{
    OptiMeas z;
    z.pos = opti.pos;
    z.psi = opti.psi;
    initializeFromOptiImpl(z);
}

inline void EKF::predict(const Vec<6>& imu, double dt)
{
    ImuMeas u;
    u.gyro = imu.segment<3>(3);
    u.accel = imu.segment<3>(0);
    predictImpl(u, dt);
}

template<typename OptiT>
void EKF::correct(const OptiT& opti)
{
    OptiMeas z;
    z.pos = opti.pos;
    z.psi = opti.psi;
    correctImpl(z);
}
