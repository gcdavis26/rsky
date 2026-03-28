#pragma once

#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include "common/MathUtils.h"

// Define a struct to hold all motor task data safely
struct MotorState {
    Vec<4> pwmCmd = Vec<4>::Zero();
    double arm_switch_pwm = 0.0;
};

// Template class to accept either MotorDriver or MotorModel
template <typename MotorType>
class MotorTask {
public:
    MotorTask(MotorType& motor);
    ~MotorTask();

    // Copy latest state from main loop
    void updateState(const Vec<4>& pwmCmd, double arm_switch_pwm);

    // Run the motor background loop
    void loop();

    // Stop background thread safely
    void stop();

    // Get arm status for telemetry
    bool isArmed() const;

private:
    MotorType& motor_;
    std::atomic<bool> isArmedCached_{false};
    MotorState current_state_;
    std::mutex state_mutex_;
    std::atomic<bool> running_{true};

    // Timer for the 5-second arming rule
    double armTime_ = 0.0;
    std::chrono::steady_clock::time_point last_time_;
};
