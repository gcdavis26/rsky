#include "estimation/AHRS.h" 


static inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

AHRS::AHRS()
    : AHRSbias(Vec<6>::Zero()),
    bg(Vec<3>::Zero()),
    q(Eigen::Quaterniond::Identity())
{
}

AHRS::AHRS(const Vec<6>& bias)
    : AHRSbias(bias),
    bg(Vec<3>::Zero()),
    q(Eigen::Quaterniond::Identity())
{
}

void AHRS::initializeFromAccel(const Vec<3>& accel) {
    Vec<3> a = accel - AHRSbias.segment<3>(3);
    double r = 0.0, p = 0.0;
    accelToAttitude(a, r, p);
    initialize(r, p, 0.0);
}

void AHRS::initialize(double roll, double pitch, double yaw) {
    double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);

    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;
    q.normalize();
    bg.setZero();
}

void AHRS::update(const Vec<3>& accel, const Vec<3>& gyro, double dt) {
    if (!(std::isfinite(dt) && dt > 0.0)) return;

    const Vec<3> a = accel - AHRSbias.segment<3>(3);

    Vec<3> omega_dot = Vec<3>::Zero();

    if (!have_prev_gyro) {
        have_prev_gyro = true;
        gyro_prev = gyro;
        omega_dot_prev.setZero();
        omega_dot.setZero();
    }
    else {
        const Vec<3> raw = (gyro - gyro_prev) / dt;
        gyro_prev = gyro;

        omega_dot = omega_dot_alpha * omega_dot_prev + (1.0 - omega_dot_alpha) * raw;
        omega_dot_prev = omega_dot;
    }

    const Vec<3> a_corr = a - omega_dot.cross(r_IMU) - gyro.cross(gyro.cross(r_IMU));

    const double amag = a.norm();
    const bool accel_ok = std::isfinite(amag) && std::abs(amag - g) <= (gate_g * g);

    Vec<3> e = Vec<3>::Zero();
    if (accel_ok && amag > 1e-6) {
        const Vec<3> a_hat = a / amag;

        // Predicted gravity direction in body frame = R^T * [0,0,1]
        // = 3rd row of R (body-to-NED rotation matrix)
        const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
        Vec<3> v_hat;
        v_hat(0) = -2.0 * (qx * qz - qw * qy);
        v_hat(1) = -2.0 * (qy * qz + qw * qx);
        v_hat(2) = -(1.0 - 2.0 * (qx * qx + qy * qy));

        e = a_hat.cross(v_hat);
    }

    bg += (ki_acc * e) * dt;

    const Vec<3> omega = gyro - bg + (kp_acc * e);

    // q_dot = 0.5 * q ⊗ [0, omega]
    Eigen::Quaterniond omega_q;
    omega_q.w() = 0.0;
    omega_q.vec() = omega;
    Eigen::Quaterniond dq = q * omega_q;
    q.coeffs() += dq.coeffs() * (0.5 * dt);
    q.normalize();
}

Vec<3> AHRS::euler() const {
    const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    const double phi = std::atan2(2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy));
    const double sinp = clampd(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
    const double theta = std::asin(sinp);
    const double psi = std::atan2(2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));

    Vec<3> rpy;
    rpy << phi, theta, wrapToPi(psi);
    return rpy;
}

void AHRS::accelToAttitude(const Vec<3>& accel, double& roll, double& pitch) {
    const double ax = accel(0), ay = accel(1), az = accel(2);
    roll = std::atan2(-ay, -az);
    pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));
}
