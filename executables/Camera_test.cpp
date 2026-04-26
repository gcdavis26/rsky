#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <array>
#include <Eigen/Dense>

#include "mocap/mocapHandler.h"
#include "sensors/ThermalCamera.h"

double clampValue(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

int main() {
    std::cout << std::fixed << std::setprecision(4);

    // Initialize Mocap Handler
    MocapHandler mocap;
    if (!mocap.init()) {
        std::cout << "Mocap failed to initialize." << std::endl;
    }

    // Initialize I2C Thermal Camera
    ThermalCamera camera;
    
    // Camera class will return error if failed to initialise
    if (!camera.init()) {
        std::cout << "Thermal Camera failed to initialize." << std::endl;
    }

    std::array<double, 768> thermal_frame;

    // Main 32Hz Processing Loop
    while (true) {
        auto start_time = std::chrono::steady_clock::now();

        // Pull drone state from mocap
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

        // Pull the latest 32x24 grid of temperatures from the hardware
        bool thermSuccess = camera.getFrame(thermal_frame);

        std::cout << "Pos [N E D]: " << pos(0) << " " << pos(1) << " " << pos(2) << " | "
                  << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << " | ";
        
        // Process the frame and output to console
        if (thermSuccess) {
            std::cout << "Therm Center Temp: " << thermal_frame[384] << std::endl;
        } else {
            std::cout << "Therm Read Failed" << std::endl;
        }

        // Yield CPU for the remainder of the 31.25ms (32Hz) time budget
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Sleep until 28ms passed from start of execution
        if (elapsed < std::chrono::milliseconds(28)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(28) - elapsed);
        }
    }
    
    return 0;
}