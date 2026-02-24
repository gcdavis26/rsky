#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <algorithm>

static constexpr double PI = 3.141592653589;
static constexpr double g = 9.81;

static constexpr double mass = 1.0;

template<int R, int C>
using Mat = Eigen::Matrix<double, R, C>;

template<int N>
using Vec = Eigen::Matrix<double, N, 1>;

template<int N>
using Vecf = Eigen::Matrix<float, N, 1>;

double clamp(double x, double lo, double hi);

double wrapToPi(double angle_rad);

Mat<3,3> RotB2N(double phi, double theta, double psi);

Vec<3> eulerRates_ZYX(double phi, double theta, const Vec<3>& omega_b);

Mat<3, 3> T_euler(double phi, double th);

Vec<3> wrapAngles(const Vec<3>& angles);

Vec<4> normPWM(const Vec<4>& rawPWM);

double softplus(double z);


