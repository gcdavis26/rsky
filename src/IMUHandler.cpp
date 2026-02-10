#include "IMUHandler.h"
#include <stdio.h>

IMUHandler::IMUHandler() {
    // initialize() in LSM9DS1.cpp needs to be changed to low range settings
    lsm.initialize();
}

Eigen::Matrix<double, 6, 1> IMUHandler::update() {

    lsm.update();

    float ax, ay, az;
    float gx, gy, gz;

    lsm.read_accelerometer(&ax, &ay, &az);
    lsm.read_gyroscope(&gx, &gy, &gz);

    Eigen::Matrix<double, 6, 1> data;
    data << gx, gy, gz, ax, ay, az;

    // Returned as a 6D eigen matrix containing acc and gyro vals
    return data;
}