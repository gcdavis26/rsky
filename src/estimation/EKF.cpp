// EKF.cpp
#include "estimation/EKF.h"

EKF::EKF() {
    // Build continuous-time Qc (12x12): [n_g; n_a; n_ba; n_bw]
    Qc.setZero();
    Qc.diagonal() <<
        sig_g * sig_g, sig_g* sig_g, sig_g* sig_g,
        sig_acc* sig_acc, sig_acc* sig_acc, sig_acc* sig_acc,
        sig_ba_walk* sig_ba_walk, sig_ba_walk* sig_ba_walk, sig_ba_walk* sig_ba_walk,
        sig_bw_walk* sig_bw_walk, sig_bw_walk* sig_bw_walk, sig_bw_walk* sig_bw_walk;

    Rpos = (sig_pos * sig_pos) * Mat<3, 3>::Identity();
    Rpsi = (sig_psi * sig_psi);

    x_est.setZero();
    P.setZero();
}

void EKF::initializeFromOpti(const OptiSim::OptiMeas& opti) {
    const double phi0 = 0.0;
    const double th0 = 0.0;
    const double psi0 = wrapToPi(opti.psi);

    x_est.setZero();
    x_est(PHI) = phi0;
    x_est(THETA) = th0;
    x_est(PSI) = psi0;

    // If opti measures an offset point: p_cg = z_pos - Rnb*r_OPTI
    const Mat<3, 3> Rnb0 = RotB2N(phi0, th0, psi0);
    const Vec<3> p_cg = opti.pos - Rnb0 * r_OPTI;

    x_est(PN) = p_cg(0);
    x_est(PE) = p_cg(1);
    x_est(PD) = p_cg(2);

    // keep P as-is (tune later)
    have_prev_gyro = false;
    gyro_prev.setZero();
    omega_dot_prev.setZero();
}

void EKF::predict(const ImuSim::ImuMeas& imu, double dt) {
    // omega_dot estimate
    Vec<3> omega_dot = Vec<3>::Zero();

    if (!have_prev_gyro) {
        have_prev_gyro = true;
        gyro_prev = imu.gyro;
        omega_dot_prev.setZero();
        omega_dot.setZero();
    }
    else {
        const Vec<3> raw = (imu.gyro - gyro_prev) / dt;
        gyro_prev = imu.gyro;

        omega_dot = omega_dot_alpha * omega_dot_prev + (1.0 - omega_dot_alpha) * raw;
        omega_dot_prev = omega_dot;
    }

    // state integration
    const Vec<NX> k1 = f_nonlin(x_est, imu, omega_dot);
    Vec<NX> x_pred = x_est + dt * k1;

    // wrap rpy
    x_pred.template segment<3>(PHI) = wrapAngles(x_pred.template segment<3>(PHI));
    
    // Covariance propagation 
    const Mat<NX, NX> F = computeF_numeric(x_est, imu, omega_dot, k1);
    const Mat<NX, 12> G = computeG(x_est);

    const Mat<NX, NX> Pk = P;

    const Mat<NX, NX> FP = F * Pk;
    
    const Mat<NX, NX> Pdot1 = FP + FP.transpose() + G * Qc * G.transpose() * dt;
    const Mat<NX, NX> P_pred = Pk + dt * Pdot1;
    P = P_pred;
    
    x_est = x_pred;
}

void EKF::correct(const OptiSim::OptiMeas& opti) {
    const Mat<NX, NX> I = Mat<NX, NX>::Identity();

    // z = [pos_ned; psi]
    Vec<4> z;
    z.template head<3>() = opti.pos;
    z(3) = wrapToPi(opti.psi);

    // h(x) = [h_pos(x); h_psi(x)]
    Vec<4> h;
    h.template head<3>() = h_pos(x_est);
    h(3) = h_psi(x_est);

    // residual (wrap yaw residual)
    Vec<4> r = z - h;
    r(3) = wrapToPi(r(3));

    // H = [Hpos; Hpsi]
    Mat<4, NX> H = Mat<4, NX>::Zero();
    H.template topRows<3>() = computeHpos(x_est);
    H(3, PSI) = 1.0;

    // R = blkdiag(Rpos, Rpsi)
    Mat<4, 4> R = Mat<4, 4>::Zero();
    R.template topLeftCorner<3, 3>() = Rpos;
    R(3, 3) = Rpsi;

    // S, K
    const Mat<4, 4> S = H * P * H.transpose() + R;
    const Mat<NX, 4> K = P * H.transpose() * S.inverse();

    // state update
    x_est = x_est + K * r;

    // Joseph form covariance update
    const Mat<NX, NX> A = (I - K * H);
    P = A * P * A.transpose() + K * R * K.transpose();

    // wrap rpy
    x_est.template segment<3>(PHI) = wrapAngles(x_est.template segment<3>(PHI));
}

// ------------------- dynamics f(x) -------------------
Vec<EKF::NX> EKF::f_nonlin(const Vec<NX>& x,
    const ImuSim::ImuMeas& imu,
    const Vec<3>& omega_dot) const
{
    Vec<NX> f = Vec<NX>::Zero();

    const double phi = x(PHI);
    const double th = x(THETA);
    const double psi = x(PSI);

    const Vec<3> ba = x.template segment<3>(BAX);
    const Vec<3> bw = x.template segment<3>(BP);

    const Vec<3> omega_m = imu.gyro;
    const Vec<3> acc_m = imu.accel;

    const Vec<3> omega = omega_m - bw;

    // a_corr = acc_m - ba - cross(omega_dot, r_IMU) - cross(omega, cross(omega, r_IMU))
    const Vec<3> a_corr =
        (acc_m - ba)
        - omega_dot.cross(r_IMU)
        - omega.cross(omega.cross(r_IMU));

    // Euler rates
    const Mat<3, 3> T = T_euler(phi, th);
    const Vec<3> eul_dot = T * omega;

    // p_dot = v
    const Vec<3> v = x.template segment<3>(VN);

    // v_dot = Rnb * a_corr + g_n
    const Mat<3, 3> Rnb = RotB2N(phi, th, psi); // body->NED
    Vec<3> g_n; g_n << 0.0, 0.0, g;      // use your constant if you have one

    const Vec<3> v_dot = Rnb * a_corr + g_n;

    f.template segment<3>(PHI) = eul_dot;
    f.template segment<3>(PN) = v;
    f.template segment<3>(VN) = v_dot;

    // bias random walks handled via process noise (f = 0 for biases)
    return f;
}

// ------------------- F numeric (central diff) -------------------
Mat<EKF::NX, EKF::NX> EKF::computeF_numeric(const Vec<NX>& x,
    const ImuSim::ImuMeas& imu,
    const Vec<3>& omega_dot,
    const Vec<NX>& k1) const
{
    Mat<NX, NX> F = Mat<NX, NX>::Zero();

    for (int j = 0; j < 9; j++) {
        Vec<NX> dx = Vec<NX>::Zero();
        dx(j) = eps_F;

        const Vec<NX> f1 = f_nonlin(x + dx, imu, omega_dot);

        F.col(j) = (f1 - k1) / (eps_F);
    }

    return F;
}

// ------------------- G (matches MATLAB structure) -------------------
Mat<EKF::NX, 12> EKF::computeG(const Vec<NX>& x) const {
    Mat<NX, 12> G = Mat<NX, 12>::Zero();

    const double phi = x(PHI);
    const double th = x(THETA);
    const double psi = x(PSI);

    const Mat<3, 3> T = T_euler(phi, th);
    const Mat<3, 3> Rnb = RotB2N(phi, th, psi);

    // gyro noise -> Euler rates through T
    G.template block<3, 3>(PHI, 0) = T;

    // accel noise -> velocity through Rnb
    G.template block<3, 3>(VN, 3) = Rnb;

    // bias random walks
    G.template block<3, 3>(BAX, 6) = Mat<3, 3>::Identity();
    G.template block<3, 3>(BP, 9) = Mat<3, 3>::Identity();

    return G;
}

// ------------------- measurement model -------------------
Vec<3> EKF::h_pos(const Vec<NX>& x) const {
    const double phi = x(PHI);
    const double th = x(THETA);
    const double psi = x(PSI);

    const Mat<3, 3> Rnb = RotB2N(phi, th, psi);

    Vec<3> p_cg;
    p_cg << x(PN), x(PE), x(PD);

    return p_cg + Rnb * r_OPTI;
}

double EKF::h_psi(const Vec<NX>& x) const {
    return x(PSI);
}

// ------------------- Hpos numeric (central diff) -------------------
Mat<3, EKF::NX> EKF::computeHpos(const Vec<NX>& x) const {
    Mat<3, NX> H = Mat<3, NX>::Zero();

    for (int j = 0; j < 9; j++) {
        Vec<NX> dx = Vec<NX>::Zero();
        dx(j) = eps_H;

        const Vec<3> h1 = h_pos(x + dx);
        const Vec<3> h0 = h_pos(x - dx);

        H.col(j) = (h1 - h0) / (2.0 * eps_H);
    }

    return H;
}
