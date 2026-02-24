#pragma once
#include <Eigen/Dense>
#include <random>

//making code cleaner
using Vector2d = Eigen::Vector2d;
using Matrix2d = Eigen::Matrix2d;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Vector4d = Eigen::Vector4d;
using Matrix4d = Eigen::Matrix4d;
using Vector10d = Eigen::Matrix<double, 10, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Vector15d = Eigen::Matrix<double, 15, 1>;
using Matrix15d = Eigen::Matrix<double, 15, 15>;



Matrix3d dcmI_B(double phi, double theta, double psi);
Vector15d get_xdot(const Vector15d& x, const Vector3d& g, const Vector3d& a_body_measured, const Vector3d& omega);
Vector12d get_dynamics(const Vector12d& x, const Vector3d& g, double m, const Vector3d& inertias, double thrust, const Vector3d& moments);
Matrix15d jacobian(const Vector15d& x, const Vector3d& g, const Vector3d& a_body_measured, const Vector3d& omega);
Eigen::Matrix<double, 15, 12> noise_coupling(const Vector15d& x);
Vector3d sim_imu_accels(const Vector12d& x_true, const Vector3d& commanded_body_accel, const Vector3d& alpha, const Vector3d& r, const Vector3d& imunoise);
Vector3d sim_gyro_rates(const Vector12d& x_true, const Vector3d& gyronoise);
Vector4d sim_measurement(const Vector4d& x_true_measured, const Vector4d& m_noise);
Vector12d noise12d();
Vector4d noise4d();
double wrapPi(double angle);
double saturate(double command, double saturation, bool ismax = true);
double clamp(double x, double lo, double hi);
Vector4d throttle2pwm(Eigen::Vector4d throttles);
