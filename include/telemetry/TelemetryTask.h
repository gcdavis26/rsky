#pragma once

#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include "telemetry/udp_sender.h"
#include "sensors/BatteryHandler.h"
#include "vision/WildfireDetection.h"

// Define a struct to hold all telemetry data safely
struct TelemetryState {
    double t = 0.0;
    double dt = 0.0;
    double Hz = 0.0;
    Vec<15> navState = Vec<15>::Zero();
    Vec<3> posCmd = Vec<3>::Zero();
    Vec<3> attCmd = Vec<3>::Zero();
    int phase = 0;
    int mode = 0;
    bool armed = false;
    double NIS = 0.0;
    Vec<4> res = Vec<4>::Zero();
    Vec<4> PWMcmd = Vec<4>::Zero();
};

class TelemetryTask {
public:
    // Updated constructor to accept the vision buffer
    TelemetryTask(UdpSender& udp_sender, VisionGridBuffer& vision_buf);
    ~TelemetryTask();

    // Copy latest state from main loop
    void updateState(const TelemetryState& new_state);

    // Run the telemetry background loop
    void loop();

    // Stop background thread safely
    void stop();

private:
    UdpSender& udp_;
    VisionGridBuffer& vision_buffer_;
    BatteryHandler battery_handler_;
    TelemetryState current_state_;
    std::mutex state_mutex_;
    std::atomic<bool> running_{true};
};