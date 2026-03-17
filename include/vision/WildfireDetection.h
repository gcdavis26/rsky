#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <optional>

// Thread-safe buffer to pass the 6-DOF EKF state TO the camera thread
class StateBuffer {
private:
    Eigen::Matrix<double, 6, 1> state_vector = Eigen::Matrix<double, 6, 1>::Zero();
    std::mutex mtx;

public:
    // Called by main control loop
    void update(const Eigen::Matrix<double, 6, 1>& new_state) {
        std::lock_guard<std::mutex> lock(mtx);
        state_vector = new_state;
    }

    // Called by the thermal camera thread
    Eigen::Matrix<double, 6, 1> getLatest() {
        std::lock_guard<std::mutex> lock(mtx);
        return state_vector;
    }
};

// Thread-safe buffer to pass the target coordinates FROM the camera thread
class HotspotBuffer {
private:
    // Defaults to empty (nullopt) until the camera thread spots something
    std::optional<Eigen::Vector2d> target_location;
    std::mutex mtx;

public:
    // Called by the thermal camera thread when a fire is confirmed
    void update(const Eigen::Vector2d& new_target) {
        std::lock_guard<std::mutex> lock(mtx);
        target_location = new_target;
    }

    // Called by the main control loop to check for active targets
    std::optional<Eigen::Vector2d> getLatest() {
        std::lock_guard<std::mutex> lock(mtx);
        return target_location;
    }
};

// The thread task signature, now accepting both the input and output buffers
void wildfireDetectionTask(StateBuffer& shared_state, HotspotBuffer& shared_targets);