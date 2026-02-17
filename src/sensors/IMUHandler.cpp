#include "IMUHandler.h"
#include <stdio.h>

IMUHandler::IMUHandler() {
    // initialize() in LSM9DS1.cpp needs to be changed to low range settings
    lsm.initialize();
}

void IMUHandler::update() {

    lsm.update();

    float ax, ay, az;
    float gx, gy, gz;

    lsm.read_accelerometer(&ax, &ay, &az);
    lsm.read_gyroscope(&gx, &gy, &gz);

    imu.accel << ax, ay, az;
    imu.gyro << gx, gy, gz;

}