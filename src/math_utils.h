#pragma once
#include <Eigen/Dense>
#include <random>

Eigen::Matrix3d dcmI_B(double phi, double theta, double psi);
Eigen::Matrix<double, 15, 1> get_xdot(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d g, Eigen::Vector3d a_body_measured, Eigen::Vector3d omega);
Eigen::Matrix<double, 12, 1> get_dynamics(Eigen::Matrix<double, 12, 1> x, Eigen::Vector3d g, double m, Eigen::Vector3d inertias, double thrust, Eigen::Vector3d moments);
Eigen::Matrix<double, 15, 15> jacobian(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d g, Eigen::Vector3d a_body_measured, Eigen::Vector3d omega);
Eigen::Matrix<double, 15, 12> noise_coupling(Eigen::Matrix<double, 15, 1> x);
Eigen::Vector3d sim_imu_accels(Eigen::Matrix<double, 12, 1> x_true, Eigen::Vector3d commanded_body_accel, Eigen::Vector3d alpha, Eigen::Vector3d r, Eigen::Vector3d imunoise);
Eigen::Vector3d sim_gyro_rates(Eigen::Matrix<double, 12, 1> x_true, Eigen::Vector3d gyronoise);
Eigen::Vector3d sim_measurement(Eigen::Vector3d x_true_measured, Eigen::Vector3d m_noise);
Eigen::Matrix<double, 12, 1> noise12d();
Eigen::Vector3d noise3d();
double wrapPi(double angle);
double saturate(double command, double saturation, bool ismax = true);


