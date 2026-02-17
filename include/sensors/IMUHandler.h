
#ifndef IMUHANDLER_H
#define IMUHANDLER_H

#include "Navio2/LSM9DS1.h"
#include "common/MathUtils.h"
#include <Eigen/Dense>

class IMUHandler {
public:
    // Constructor handles probing and initialization
    IMUHandler();
    
    struct imuData {
        Vec<3> accel = Vec<3>::Zero();
        Vec<3> gyro = Vec<3>::Zero();
    };

    void update();

    imuData imu;

private:
    LSM9DS1 lsm;
};

#endif