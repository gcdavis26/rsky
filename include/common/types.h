#pragma once

#include <Eigen/Dense>

namespace gnc {

// Common vector/matrix types used everywhere
typedef Eigen::Vector3d Vec3;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Vector2d Vec2;

// Global constants (plain doubles, no constexpr)
static const double GRAVITY = 9.80665;   // [m/s^2]
static const double PI      = 3.141592653589793;
static const double MASS 	= 1.0;
} // namespace gnc
