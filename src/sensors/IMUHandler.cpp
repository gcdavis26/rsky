#include "sensors/IMUHandler.h"
#include <stdio.h>
#include <chrono>

IMUHandler::IMUHandler() {
    // initialize() in LSM9DS1.cpp needs to be changed to low range settings
    lsm.initialize();
}

Eigen::Matrix<double, 12, 1> IMUHandler::initialize() {
    Eigen::Matrix<double, 6, 1> sum = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> sum_sq = Eigen::Matrix<double, 6, 1>::Zero();
    int count = 0;

    // Set up the timer for 5 seconds
    auto start_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::seconds(5);

    // Loop until 5 seconds have passed
    while (std::chrono::steady_clock::now() - start_time < duration) {

        update();
        Vec<6> data = Vec<6>::Zero();
        data << imu.gyro, imu.accel;

        sum += data;

        // Element-wise multiplication to accumulate the sum of squares
        sum_sq += data.cwiseProduct(data);

        count++;
    }

    Eigen::Matrix<double, 6, 1> mean = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> std_dev = Eigen::Matrix<double, 6, 1>::Zero();


    mean = sum / static_cast<double>(count);

    Eigen::Matrix<double, 6, 1> variance = (sum_sq - (sum.cwiseProduct(sum) / count)) / (count - 1);

    std_dev = variance.cwiseSqrt();


    // Initialize the 12x1 result matrix and stack the mean and std_dev into it
    Eigen::Matrix<double, 12, 1> result;
    result << mean, std_dev;

    return result;
}

void IMUHandler::update() {

    lsm.update();

    float ax, ay, az;
    float gx, gy, gz;

    lsm.read_accelerometer(&ax, &ay, &az);
    lsm.read_gyroscope(&gx, &gy, &gz);

    Eigen::Matrix<double, 6, 1> data;
    data << gy,gx, gz, ay, ax, -az;

    imu.gyro << gy, gx, -gz;
    imu.accel << ay, ax, -az;
    
}