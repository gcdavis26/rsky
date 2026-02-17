#ifndef IMUHANDLER_H
#define IMUHANDLER_H

#include "Navio2/LSM9DS1.h"
#include <Eigen/Dense>

class IMUHandler {
public:
    // Constructor handles probing and initialization
    IMUHandler();

    // Reads sensor and returns 6x1 Eigen Matrix [ax, ay, az, gx, gy, gz]
    Eigen::Matrix<double, 6, 1> update();

private:
    LSM9DS1 lsm;
};

#endif