#pragma once
#include <Eigen/Dense>
#include <random>

Eigen::Matrix3d dcmI_B(double phi, double theta, double psi);
Eigen::Matrix<double, 15, 1> get_xdot(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d g, Eigen::Vector3d a_inertial_measured, Eigen::Vector3d omega);
Eigen::Matrix<double, 15, 15> jacobian(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d g, Eigen::Vector3d a_inertial_measured, Eigen::Vector3d omega);
Eigen::Matrix<double, 15, 12> noise_coupling(Eigen::Matrix<double, 15, 1> x);
double wrapPi(double angle);


