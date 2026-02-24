#pragma once
#include <Eigen/Dense>

int openPort(void);

typedef Eigen::Matrix<double, 5, 1> Vector5d;
Vector5d readDatalink(void);