#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <array>

#include "mocap/mocapHandler.h"
#include "sensors/ThermalCamera.h"

double clampValue(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

int main() {
    std::cout << std::fixed << std::setprecision(4);

    MocapHandler mocap;
    if (!mocap.init()) {
        std::cout << "Mocap failed to initialize." << std::endl;
    }

    ThermalCamera thermCam(0x33, 1);
    if (!thermCam.init()) {
        std::cout << "Thermal Camera failed to initialize." << std::endl;
    }

    std::array<double, 768> thermalBuffer;
    double target_dt = 1.0 / 32.0;

    while (true) {
        auto start = std::chrono::steady_clock::now();

        mocap.update();
        Vec<3> pos = mocap.getPosition();
        
        Eigen::Quaternionf q = mocap.getQuaternion();
        double w = q.w();
        double x = q.x();
        double y = q.y();
        double z = q.z();
        
        double yaw = std::atan2(
            2.0 * (w * z + x * z),
            1.0 - 2.0 * (y * y + z * z)
        );
        
        double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        
        double sinp = 2.0 * (w * y - z * x);
        sinp = clampValue(sinp, -1.0, 1.0);
        double pitch = std::asin(sinp);

        bool thermSuccess = thermCam.getFrame(thermalBuffer);

        std::cout << "Pos [N E D]: " << pos(0) << " " << pos(1) << " " << pos(2) << " | "
                  << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << " | ";
        
        if (thermSuccess) {
            std::cout << "Therm Center Temp: " << thermalBuffer[384] << std::endl;
        } else {
            std::cout << "Therm Read Failed" << std::endl;
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end - start;
        double sleep_s = target_dt - diff.count();
        
        if (sleep_s > 0.0) {
            int sleep_us = (int)(sleep_s * 1e6);
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }
    }
    
    return 0;
}