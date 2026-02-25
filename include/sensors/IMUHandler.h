#ifndef IMUHANDLER_H
#define IMUHANDLER_H

#include "Navio2/LSM9DS1.h"
#include <Eigen/Dense>
#include "common/MathUtils.h"

class IMUHandler {
public:
    // Constructor handles probing and initialization
    IMUHandler();

    Eigen::Matrix<double, 12, 1> initialize();

    // Reads sensor and returns 6x1 Eigen Matrix [ax, ay, az, gx, gy, gz]

    struct imuOut {
        Vec<3> gyro = Vec<3>::Zero();
        Vec<3> accel = Vec<3>::Zero();
    };

    imuOut imu;

    void update();

private:
    LSM9DS1 lsm;
};

#endif