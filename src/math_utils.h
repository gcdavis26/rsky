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
Vector15d get_xdot(Vector15d x, Eigen::Vector3d g, Vector3d a_body_measured, Vector3d omega);
Vector12d get_dynamics(Vector12d x, Vector3d g, double m, Vector3d inertias, double thrust, Vector3d moments);
Matrix15d jacobian(Vector15d x, Vector3d g, Vector3d a_body_measured, Vector3d omega);
Eigen::Matrix<double, 15, 12> noise_coupling(Vector15d x);
Vector3d sim_imu_accels(Vector12d x_true, Vector3d commanded_body_accel, Vector3d alpha, Vector3d r, Vector3d imunoise);
Vector3d sim_gyro_rates(Vector12d x_true, Vector3d gyronoise);
Vector3d sim_measurement(Vector3d x_true_measured, Vector3d m_noise);
Vector12d noise12d();
Vector3d noise3d();
double wrapPi(double angle);
double saturate(double command, double saturation, bool ismax = true);
double clamp(double x, double lo, double hi);

