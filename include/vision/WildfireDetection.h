#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <optional>
#include <vector>

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

struct VisionData {
    std::vector<int> hot_cells;
    std::vector<int> blob_cells;
    bool is_fresh = false;
};

class VisionGridBuffer {
private:
    VisionData data;
    std::mutex mtx;

public:
    // Called by the vision thread
    void update(const std::vector<int>& hot, const std::vector<int>& blob) {
        std::lock_guard<std::mutex> lock(mtx);
        data.hot_cells = hot;
        data.blob_cells = blob;
        data.is_fresh = true;
    }

    // Called by TelemetryTask. Returns true and clears the flag if data was new.
    bool consume(std::vector<int>& out_hot, std::vector<int>& out_blob) {
        std::lock_guard<std::mutex> lock(mtx);
        if (data.is_fresh) {
            out_hot = data.hot_cells;
            out_blob = data.blob_cells;
            data.is_fresh = false;
            return true;
        }
        return false;
    }
};

// The thread task signature, now accepting all THREE buffers
void wildfireDetectionTask(StateBuffer& shared_state, HotspotBuffer& shared_targets, VisionGridBuffer& vision_telemetry);