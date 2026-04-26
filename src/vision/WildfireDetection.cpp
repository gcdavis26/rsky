#include "ThermalCamera.h"
#include "SearchGrid.h"
#include "WildfireDetection.h" // Contains StateBuffer and HotspotBuffer
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <unistd.h>

void wildfireDetectionTask(StateBuffer& shared_state, HotspotBuffer& shared_targets) {
    
    // Initialize I2C Thermal Camera
    ThermalCamera camera;
    
    // Camera class will return error if failed to initialise
    if (!camera.init()) return;

    // Initialize local memory map
    SearchGrid local_grid;
    std::array<double, 768> thermal_frame;

    double target_dt = 1.0 / 32.0;

    // Main 32Hz Processing Loop
    while (true) {
        auto start_time = std::chrono::steady_clock::now();

        // Pull the latest 32x24 grid of temperatures from the hardware
        if (camera.getFrame(thermal_frame)) {

            // Pull drone state as 6x1 Eigen Matrix (x, y, z, roll, pitch, yaw)
            Eigen::Matrix<double, 6, 1> current_state = shared_state.getLatest();

            // Process the frame. Returns true if a fully bounded fire is found.
            if (local_grid.processFrame(thermal_frame, current_state)) {

                // If a target was found, get center location
                if (auto fire_location_opt = local_grid.getCenter()) {

                    // Unwrap the std::optional into a Eigen 2x1 matrix
                    Eigen::Vector2d fire_coords = *fire_location_opt;

                    // Pass the 2x1 matrix to the main control loop via the shared buffer
                    shared_targets.update(fire_coords);
                }
            }
        }

        // --------- Loop timing: compute sleep at fixed target rate ---------
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> compute_dur = end_time - start_time;
        double compute_s = compute_dur.count();

        double sleep_s = target_dt - compute_s;
        
        if (sleep_s > 0.0) {
            // usleep for the bulk of the time, stopping 100 microseconds early
            double early_wake_s = sleep_s - 0.000100; 
            if (early_wake_s > 0.0) {
                usleep((int)(early_wake_s * 1e6));
            }

            // Spin-lock (busy wait) for the final fractions of a millisecond
            while (true) {
                auto current_time = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = current_time - start_time;
                if (elapsed.count() >= target_dt) {
                    break; 
                }
            }
        }
    }
}