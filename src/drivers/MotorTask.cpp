#include "drivers/MotorTask.h"
#include "drivers/MotorDriver.h"
#include "simulator/MotorModel.h"
#include <iostream>
#include <type_traits>

template <typename MotorType>
MotorTask<MotorType>::MotorTask(MotorType& motor) : motor_(motor), armedTime_(0.0) {
    last_time_ = std::chrono::steady_clock::now();
}

template <typename MotorType>
MotorTask<MotorType>::~MotorTask() {
    stop();
}

template <typename MotorType>
void MotorTask<MotorType>::updateState(const Vec<4>& pwmCmd, double arm_switch_pwm, double servo_pwm) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.pwmCmd = pwmCmd;
    current_state_.arm_switch_pwm = arm_switch_pwm;
    current_state_.servo_pwm = servo_pwm;
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

    if constexpr (std::is_same_v<MotorType, MotorDriver>) {
        motor_.initialize();
        std::cout << "Motors initialized. System ready to arm." << std::endl;
    }

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

        // Arming logic: arm immediately when arm switch is high.
        if (state_copy.arm_switch_pwm > 1500.0) {
            if (!motor_.isArmed()) {
                motor_.arm();
                armedTime_ = 0.0;
            }
            else {
                armedTime_ += dt;
            }
        } else {
            armedTime_ = 0.0;
            motor_.disarm();
        }

        isArmedCached_.store(motor_.isArmed());

        // Real Commands (Simulated or Real Hardware)
        if (isArmedCached_.load()) {
            if (armedTime_ < 1.0) {
                // For the first 1 second after arming, send exactly 1000 PWM
                motor_.command(Vec<4>::Constant(1000.0));
            }
            else {
                // After 1 second, send actual commands from the mixer
                motor_.command(state_copy.pwmCmd);
            }
        }

        // Sleep to maintain ~400Hz loop rate (2.5ms = 2500 microseconds) using sleep_until
        next_loop_time += std::chrono::microseconds(2500);
        std::this_thread::sleep_until(next_loop_time);
    }
}

// Explicit template instantiations
// Explicit template instantiations
#ifdef PLATFORM_LINUX
template class MotorTask<MotorDriver>;
#endif

template class MotorTask<MotorModel>;
