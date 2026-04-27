#include "telemetry/TelemetryTask.h"
#include <vector>

// Initialize the vision_buffer_ reference
TelemetryTask::TelemetryTask(UdpSender& udp_sender, VisionGridBuffer& vision_buf) 
    : udp_(udp_sender), vision_buffer_(vision_buf) {
}

TelemetryTask::~TelemetryTask() {
    stop();
}

void TelemetryTask::updateState(const TelemetryState& new_state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_ = new_state;
}

void TelemetryTask::stop() {
    running_ = false;
}

void TelemetryTask::loop() {
    while (running_) {
        // Sleep first to maintain 25Hz loop rate
        std::this_thread::sleep_for(std::chrono::milliseconds(40));

        TelemetryState state_copy;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_copy = current_state_;
        }
        const Eigen::Vector2d battery_data = battery_handler_.read_battery();
        const double battery_voltage_mv = battery_data(0);
        const double battery_current_ma = battery_data(1);

        // We use the modified sendFromSim that takes primitives
        udp_.sendFromSim(
            state_copy.t,
            state_copy.dt,
            state_copy.Hz,
            state_copy.navState,
            state_copy.posCmd,
            state_copy.phase,
            state_copy.mode,
            state_copy.attCmd,
            state_copy.armed,
            state_copy.NIS,
            state_copy.res,
            state_copy.PWMcmd,
            battery_voltage_mv,
            battery_current_ma
        );

        std::vector<int> hot_cells;
        std::vector<int> blob_cells;
        
        // Attempt to consume new data from the vision thread
        if (vision_buffer_.consume(hot_cells, blob_cells)) {
            // Only fire off a packet if the camera actually detected heat
            if (!hot_cells.empty()) {
                udp_.sendVisionData(hot_cells, blob_cells);
            }
        }
    }
}