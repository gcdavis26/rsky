#include "ThermalCamera.h"
#include "SearchGrid.h"
#include "WildfireDetection.h" // Contains StateBuffer and HotspotBuffer
#include <chrono>
#include <thread>
#include <Eigen/Dense>

void wildfireDetectionTask(StateBuffer& shared_state, HotspotBuffer& shared_targets) {
    
    // Initialize I2C Thermal Camera
    ThermalCamera camera;
    
    // Camera class will return error if failed to initialise
    if (!camera.init()) return;

    // Initialize local memory map
    SearchGrid local_grid;
    std::array<double, 768> thermal_frame;

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

        // Yield CPU for the remainder of the 31.25ms (32Hz) time budget
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Sleep until 28ms passed from start of execution
        if (elapsed < std::chrono::milliseconds(28)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(28) - elapsed);
        }
    }
}