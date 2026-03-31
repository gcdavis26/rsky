#include "telemetry/TelemetryTask.h"

TelemetryTask::TelemetryTask(UdpSender& udp_sender) : udp_(udp_sender) {
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
            state_copy.PWMcmd
        );
    }
}
