#pragma once
#include <Eigen/Dense>

int openPort(void);

typedef Eigen::Matrix<double, 8, 1> Vector8d;
Vector8d readDatalink(void);