#include "IMUHandler.h"
#include <stdio.h>

IMUHandler::IMUHandler() {
    // initialize() in LSM9DS1.cpp needs to be changed to low range settings
    lsm.initialize();
}

Eigen::Matrix<double, 6, 1> IMUHandler::initialize() {
    Eigen::Matrix<double, 6, 1> sum = Eigen::Matrix<double, 6, 1>::Zero();
    int count = 0;

    // Set up the timer for 5 seconds
    auto start_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::seconds(5);

    // Loop until 5 seconds have passed
    while (std::chrono::steady_clock::now() - start_time < duration) {

        sum += update();
        count++;
        
    }

    if (count > 0) {
        sum /= static_cast<double>(count);
    }

    return sum;
}

Eigen::Matrix<double, 6, 1> IMUHandler::update() {

    lsm.update();

    float ax, ay, az;
    float gx, gy, gz;

    lsm.read_accelerometer(&ax, &ay, &az);
    lsm.read_gyroscope(&gx, &gy, &gz);

    Eigen::Matrix<double, 6, 1> data;
    data << gy,gx, gz, ay, ax, -az;

    // Returned as a 6D eigen matrix containing acc and gyro vals
    return data;
}