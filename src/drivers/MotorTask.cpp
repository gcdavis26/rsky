#include "drivers/MotorTask.h"
#include "drivers/MotorDriver.h"
#include "simulator/MotorModel.h"
#include <iostream>

template <typename MotorType>
MotorTask<MotorType>::MotorTask(MotorType& motor) : motor_(motor), armTime_(0.0) {
    last_time_ = std::chrono::steady_clock::now();
}

template <typename MotorType>
MotorTask<MotorType>::~MotorTask() {
    stop();
}

template <typename MotorType>
void MotorTask<MotorType>::updateState(const Vec<4>& pwmCmd, double arm_switch_pwm) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.pwmCmd = pwmCmd;
    current_state_.arm_switch_pwm = arm_switch_pwm;
}

template <typename MotorType>
void MotorTask<MotorType>::stop() {
    running_ = false;
}

template <typename MotorType>
bool MotorTask<MotorType>::isArmed() const {
    return isArmedCached_.load();
}

template <typename MotorType>
void MotorTask<MotorType>::loop() {
    auto next_loop_time = std::chrono::steady_clock::now();
    last_time_ = next_loop_time;

    while (running_) {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt_duration = current_time - last_time_;
        double dt = dt_duration.count();
        last_time_ = current_time;

        MotorState state_copy;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_copy = current_state_;
        }

        // Arming logic
        if (state_copy.arm_switch_pwm > 1500.0) {
            armTime_ += dt;
            if (armTime_ >= 5.0) {
                motor_.arm();
            }
        } else {
            armTime_ = 0.0;
            motor_.disarm();
        }

        isArmedCached_.store(motor_.isArmed());

        // Real Commands (Simulated or Real Hardware)
        if (isArmedCached_.load()) {
            motor_.command(state_copy.pwmCmd); // Takes in four motors 1 2 3 4 pwmCmd
        } else {
            // motor_.disarm() internally handles wind_down() on transition
            // So we simply do nothing here while continuously disarmed
        }

        // Sleep to maintain ~400Hz loop rate (2.5ms = 2500 microseconds) using sleep_until
        next_loop_time += std::chrono::microseconds(2500);
        std::this_thread::sleep_until(next_loop_time);
    }
}

// Explicit template instantiations
template class MotorTask<MotorDriver>;
template class MotorTask<MotorModel>;
