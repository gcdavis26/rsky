#pragma once
#include <Eigen/Dense>
#include <random>

Eigen::Matrix3d dcmI_B(double phi, double theta, double psi);
Eigen::Matrix<double, 15, 1> get_xdot(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d g, Eigen::Vector3d a_inertial_measured, Eigen::Vector3d omega); //ekf
Eigen::Matrix<double, 12, 1> get_dynamics(Eigen::Matrix<double, 12, 1> x, Eigen::Vector3d g, double m, Eigen::Vector3d inertias, Eigen::Vector3d forces, Eigen::Vector3d moments);
Eigen::Matrix<double, 15, 15> jacobian(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d g, Eigen::Vector3d a_inertial_measured, Eigen::Vector3d omega);
Eigen::Matrix<double, 15, 12> noise_coupling(Eigen::Matrix<double, 15, 1> x);
double wrapPi(double angle);
double saturate(double command, double saturation, bool ismax = true);


