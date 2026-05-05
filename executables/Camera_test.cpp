#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <array>
#include <csignal> // Required for signal handling
#include <Eigen/Dense>

#include "mocap/mocapHandler.h"
#include "sensors/ThermalCamera.h"
#include "vision/SearchGrid.h"

// Global flag to safely break the loop on Ctrl+C
volatile std::sig_atomic_t keep_running = 1;

void sigint_handler(int signum) {
    (void)signum;
    keep_running = 0;
}

double clampValue(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

int main() {
    std::cout << std::fixed << std::setprecision(4);

    // Register the Ctrl+C signal handler
    std::signal(SIGINT, sigint_handler);

    // Toggle this bool to test the full SearchGrid vision processing pipeline
    bool test_vision = true;

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

    SearchGrid local_grid;
    std::array<double, 768> thermal_frame;

    bool fire_seen = false;

    std::cout << "Starting 32Hz Vision Loop. Press Ctrl+C to exit and save map." << std::endl;

    // Main 32Hz Processing Loop controlled by the signal flag
    while (keep_running) {
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

        // Pull the latest 32x24 grid of temperatures and evaluate directly
        if (camera.getFrame(thermal_frame)) {
            std::cout << "Pos [N E D]: " << pos(0) << " " << pos(1) << " " << pos(2) << " | "
                      << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << " | "
                      << "Therm Center Temp: " << thermal_frame[384];
            
            // Run full SearchGrid mapping if testing vision
            if (test_vision) {
                Eigen::Matrix<double, 6, 1> current_state;
                // Pack natively as [roll, pitch, yaw, x, y, z] to match SearchGrid indices
                current_state << roll, pitch, yaw, pos(0), pos(1), pos(2);
                
                if (local_grid.processFrame(thermal_frame, current_state)) {
                    if (auto fire_location_opt = local_grid.getCenter()) {
                        Eigen::Vector2d fire_coords = *fire_location_opt;
			fire_seen = true;
                        std::cout << " | FIRE DETECTED at [N: " << fire_coords(0) << ", E: " << fire_coords(1) << "]";
                    }
                }
            }
            std::cout << std::endl;
        } else {
            std::cout << "Therm Read Failed" << std::endl;
        }

	if(fire_seen){
		std::cout<<"Fire has been seen \n";
	}

        // Yield CPU for the remainder of the 31.25ms (32Hz) time budget
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Sleep until 28ms passed from start of execution
        if (elapsed < std::chrono::milliseconds(26)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(26) - elapsed);
        }
    }
    
    // Program has received Ctrl+C and broken out of the loop.
    std::cout << "\nExiting... Exporting thermal map to ground_state.csv" << std::endl;
    local_grid.exportToCSV("ground_state.csv");
    
    return 0;
}
