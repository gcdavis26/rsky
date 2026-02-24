// EKF.h
#pragma once

#include "common/MathUtils.h"
#include "sensors/ImuSim.h"
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

    bool init = false;

    void initializeFromOpti(const OptiSim::OptiMeas& opti);
    void predict(const ImuSim::ImuMeas& imu, double dt);
    void correct(const OptiSim::OptiMeas& opti);

    Vec<NX> getx() const { return x_est; }
    Mat<NX, NX> getP() const { return P; }
    double getHealth() const { return nisAvg; }


private:
    // state + covariance
    Vec<NX>   x_est = Vec<NX>::Zero();
    Mat<NX, NX> P = Mat<NX, NX>::Zero();
    Mat<4, 4> S = Mat<4, 4>::Zero();
    Vec<4> res = Vec<4>::Zero();

    // geometry
    Vec<3> r_IMU = Vec<3>::Zero(); // (-1.3cm,-0.9cm,-5.8cm)
    Vec<3> r_OPTI = Vec<3>::Zero();

    // process noise (continuous-time)
    double sig_g = 0.00017678;
    double sig_acc = 0.0010607;
    double sig_ba_walk = 1e-6;
    double sig_bw_walk = 1e-9;

    // measurement noise
    double sig_pos = 0.001*1.2;
    double sig_psi = 0.001*1.2;

    // numerics
    double eps_F = 1e-6;
    double eps_H = 1e-6;
    double omega_dot_alpha = 0.7;

    // omega_dot estimation
    bool  have_prev_gyro = false;
    Vec<3> gyro_prev = Vec<3>::Zero();
    Vec<3> omega_dot_prev = Vec<3>::Zero();

    // cached noise covariances
    Mat<12, 12> Qc = Mat<12, 12>::Zero();   // [ng(3) na(3) nba(3) nbw(3)]
    Mat<3, 3>   Rpos = Mat<3, 3>::Zero();
    double     Rpsi = 0.0;

    // dynamics + Jacobians
    Vec<NX>     f_nonlin(const Vec<NX>& x, const ImuSim::ImuMeas& imu, const Vec<3>& omega_dot) const;
    Mat<NX, NX>  computeF(const Vec<NX>& x, const ImuSim::ImuMeas& imu, const Vec<3>& omega_dot, const Vec<NX>& k1) const;
    Mat<NX, 12>  computeG(const Vec<NX>& x) const;

    // measurement model + Jacobian
    Vec<3>      h_pos(const Vec<NX>& x) const;
    double      h_psi(const Vec<NX>& x) const;
    Mat<3, NX>   computeHpos(const Vec<NX>& x) const;

    // Health Monitoring
    double nisAvg = 0.0;
    bool nisInit = false;

    // helpers (from your MathUtils)
    // Mat<3,3> RotB2N(double phi, double th, double psi);
    // Mat<3,3> T_euler(double phi, double th);
    // double   wrapToPi(double a);
    // Vec<3>   wrapAngles(const Vec<3>& rpy);
};
