#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>

#include "common/MathUtils.h"
#include "common/LowPass.h"
#include "control/InnerLoop.h"
#include "control/OuterLoop.h"
#include "control/QuadMixer.h"
#include "estimation/EKF.h"
#include "guidance/ModeManager.h"
#include "telemetry/udp_sender.h"
#include "telemetry/TelemetryTask.h"
#include "estimation/AHRS.h"
#include "drivers/MotorDriver.h"
#include "drivers/MotorTask.h"
#include "drivers/RCIn.h"
#include "sensors/IMUHandler.h"
#include "mocap/mocapHandler.h"


// Helper function to set priority and pin a std::thread
void configureThread(std::thread& target_thread, int priority, int core_id = -1) {
    pthread_t handle = target_thread.native_handle();

    // 1. Set Priority
    if (priority > 0) {
        sched_param sch_params;
        sch_params.sched_priority = priority;
        if (pthread_setschedparam(handle, SCHED_FIFO, &sch_params) != 0) {
            std::cerr << "Failed to set SCHED_FIFO " << priority << ". Need sudo?" << std::endl;
        }
    }

    // 2. Set Core Affinity (Pinning)
    if (core_id >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_id, &cpuset);
        if (pthread_setaffinity_np(handle, sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Failed to pin thread to core " << core_id << std::endl;
        }
    }
}

int main() {
    std::cout << std::fixed << std::setprecision(4);

    // --- Init objects (cloned from main.cpp, without TimeKeeper) ---
    ModeManager MM(false);
    OuterLoop outer;
    InnerLoop inner;
    QuadMixer mixer;
    UdpSender udp("192.168.1.2", 8080); // KINETIC 192.168.1.2
    TelemetryState ts;

    TelemetryTask telemetry_task(udp);
    std::thread telemetry_thread(&TelemetryTask::loop, &telemetry_task);

    RCIn rcin;
    rcin.initialize();

    MotorDriver motdrv;

    IMUHandler imuReal;
    Vec<12> imuStats = imuReal.initialize();
    imuStats(5) = imuStats(5) + g;
    std::cout << "IMU values initialized successfully." << std::endl;
    AHRS ahrs(imuStats.segment<6>(0));
    EKF ekf(imuStats.segment<6>(0));

    MotorTask<MotorDriver> motor_task(motdrv);
    std::thread motor_thread(&MotorTask<MotorDriver>::loop, &motor_task);
    
    // Elevate and pin the Motor Thread (Priority 80, Core 2)
    configureThread(motor_thread, 80, 2);
    std::cout << "Motor thread started. Waiting for motor initialization..." << std::endl;

    MocapHandler mocap;
    bool mocapInit = mocap.init();

    ImuLpf ekf_filter(500.0f, 160.0f);
    ImuLpf ctrl_filter(500.0f, 160.0f);
    ekf_filter.on = true;
    ctrl_filter.on = true;

    // --- State / command vars (copied from main.cpp) ---
    double lastPrint = 0.0;
    const double printDt = 1.0;

    Vec<3> momentsCmd = Vec<3>::Zero();
    Vec<4> thrustCmd = Vec<4>::Zero();
    Vec<4> wrenchCmd = Vec<4>::Zero();
    Vec<4> pwmCmd = Vec<4>::Zero();
    Vec<6> rcPWM = Vec<6> ::Zero();
    Vec<4> throttleTest = Vec<4>::Zero();
    Vec<6> raw;

    Vec<3> manVel = Vec<3>::Zero();
    double manPsi = 0.0;

    int step = 0;
    double Hz = 0.0;

    bool autopilot = false;
    bool printOn = false;

    double NIS = 4.0;
    bool ekfHealthy = false;
    double ekfbadTimer = 0.0;
    double ekfgoodTimer = 0.0;

    // --- New timing model (no TimeKeeper): fixed loop period per run ---
    using clock_t = std::chrono::steady_clock;
    auto start_time = clock_t::now();
    auto last_time = start_time;

    // Fixed target period (change this value to adjust control rate between runs)
    double target_dt = 1.0 / 650.0;     // 650 Hz target loop rate
    double avg_sleep = 0.0;
    double avg_compute = 0.0;
    const double ema_alpha = 0.01;
    int overrun_count = 0;

    // task-rate accumulators (approximate original TimeKeeper rates)
    double t = 0.0;
    double acc_imu = 0.0;
    double acc_opti = 0.0;
    double acc_navPred = 0.0;
    double acc_navCorr = 0.0;
    double acc_MM = 0.0;
    double acc_keys = 0.0;
    double acc_conOuter = 0.0;
    double acc_AHRS = 0.0;
    double acc_conInner = 0.0;
    double acc_tele = 0.0;

    // rough guesses for task periods (seconds) – adjust if you know exact values
    const double rate_imu = 1.0 / 650.0;
    const double rate_opti = 1.0 / 120.0;
    const double rate_navPred = 1.0 / 650.0;
    const double rate_navCorr = 1.0 / 120.0;
    const double rate_MM = 1.0 / 100.0;
    const double rate_keys = 1.0 / 100.0;
    const double rate_conOuter = 1.0 / 200.0;
    const double rate_AHRS = 1.0 / 650.0;
    const double rate_conInner = 1.0 / 650.0;
    const double rate_tele = 1.0 / 100.0;

    // Elevate and pin the current Main Thread (Priority 90, Core 3)
    pthread_t main_handle = pthread_self();
    
    sched_param main_params;
    main_params.sched_priority = 90;
    pthread_setschedparam(main_handle, SCHED_FIFO, &main_params);

    cpu_set_t main_cpuset;
    CPU_ZERO(&main_cpuset);
    CPU_SET(3, &main_cpuset);
    pthread_setaffinity_np(main_handle, sizeof(cpu_set_t), &main_cpuset);

    while (true) {
        auto loop_start = clock_t::now();
        std::chrono::duration<double> dt_ch = loop_start - last_time;
        double dt = dt_ch.count();
        last_time = loop_start;

        t = std::chrono::duration<double>(loop_start - start_time).count();
        step++;

        // Hz estimate (simple EMA)
        double inst_Hz = dt > 0.0 ? 1.0 / dt : 0.0;
        const double Hz_alpha = 0.1;
        Hz = (1.0 - Hz_alpha) * Hz + Hz_alpha * inst_Hz;

        // update accumulators
        acc_imu += dt;
        acc_opti += dt;
        acc_navPred += dt;
        acc_navCorr += dt;
        acc_MM += dt;
        acc_keys += dt;
        acc_conOuter += dt;
        acc_AHRS += dt;
        acc_conInner += dt;
        acc_tele += dt;

        // ---------------- IMU ----------------
        if (acc_imu >= rate_imu) {
            imuReal.update();
            raw.segment<3>(0) = imuReal.imu.accel;
            raw.segment<3>(3) = imuReal.imu.gyro;
            ekf_filter.update(raw);
            ctrl_filter.update(raw);
            acc_imu -= rate_imu;
        }

        // ---------------- Mocap ----------------
        if (acc_opti >= rate_opti) {
            mocap.update();
            acc_opti -= rate_opti;
        }

        // ---------------- EKF ----------------
        if (!ekf.init && mocapInit) {
            ekf.initializeFromOpti(mocap.opti);
            ekf.init = true;
        }

        if (acc_navPred >= rate_navPred) {
            ekf.predict(ekf_filter.output(), acc_navPred);
            acc_navPred = 0.0;
        }

        if ((acc_navCorr >= rate_navCorr) && mocap.m_valid) {
            ekf.correct(mocap.opti);

            NIS = ekf.getHealth();
            if (NIS > 13.28) {
                ekfbadTimer += acc_navCorr;
                ekfgoodTimer = 0.0;
            } else {
                ekfbadTimer = 0.0;
                ekfgoodTimer += acc_navCorr;
            }

            if (ekfbadTimer > 5.0) {
                ekfHealthy = false;
            } else if (ekfgoodTimer > 1.0) {
                ekfHealthy = true;
            }

            acc_navCorr = 0.0;
        }
        Vec<15> navState = ekf.getx();

        // ---------------- Mode Manager ----------------
        if ((acc_MM >= rate_MM) && ekfHealthy && motor_task.isArmed()) {
            MM.in.state = navState;
            MM.in.dt = dt;
            MM.in.detected = false;
            MM.update();
            acc_MM = 0.0;
        }

        // ---------------- Manual RC Controls ----------------
        if (acc_keys >= rate_keys) {
            Vec<3> rcVel = Vec<3>::Zero();
            double rcPsi = 0.0;

            rcPWM = rcin.read_ppm_vector();

            if (rcPWM(5) > 1750) {
                // drop stuff
            } else if (rcPWM(5) > 1250) {
                autopilot = true;
            } else {
                autopilot = false;
            }

            Vec<4> normalizedPWM = normPWM(rcPWM.segment<4>(0));

            rcVel(0) = normalizedPWM(1);
            rcVel(1) = normalizedPWM(0);
            rcVel(2) = -normalizedPWM(2);

            rcPsi = normalizedPWM(3);

            acc_keys = 0.0;

            if (motor_task.isArmed()) {
                manPsi = rcPsi;
                manVel = rcVel;
            }
        }

        // ---------------- Outer Loop ----------------
        if (acc_conOuter >= rate_conOuter) {
            if (autopilot) {
                outer.in.state = navState.segment<6>(3);
                outer.in.posCmd = MM.out.posCmd;
                outer.in.psi = navState(2);
                outer.in.mode = MM.out.mode;
                outer.in.dt = acc_conOuter;
                outer.in.arm = motor_task.isArmed();
                outer.update();
            } else {
                outer.in.state = navState.segment<6>(3);
                outer.in.posCmd = navState.segment<3>(3);
                outer.in.psi = navState(2);
                outer.in.mode = ModeManager::NavMode::Manual;
                outer.in.dt = acc_conOuter;
                outer.in.arm = motor_task.isArmed();
                outer.in.velCmd = manVel;
                outer.update();
                outer.out.attCmd(2) = navState(2);
            }
            acc_conOuter = 0.0;
        }

        // ---------------- AHRS ----------------
        if (!ahrs.init) {
            ahrs.initialize(0.0, 0.0, 0.0);
            ahrs.init = true;
        }
        if (acc_AHRS >= rate_AHRS) {
            ahrs.update(
                ctrl_filter.output().segment<3>(0) - imuStats.segment<3>(3),
                ctrl_filter.output().segment<3>(3) - imuStats.segment<3>(0),
                acc_AHRS);
            acc_AHRS = 0.0;
        }

        Vec<3> AHRSAtt = ahrs.euler();
        Vec<3> attManual;

        // ---------------- Inner Loop ----------------
        if (acc_conInner >= rate_conInner) {
            attManual << 10 * PI / 180 * manVel(1), -10 * PI / 180 * manVel(0), navState(2);
            manPsi = manPsi * 20 * PI / 180;

            if (autopilot && ekfHealthy) {
                momentsCmd =
                    inner.computeWrench(
                        outer.out.attCmd,
                        0.0,
                        navState.segment<3>(0),
                        ctrl_filter.output().segment<3>(3) - imuStats.segment<3>(0),
                        acc_conInner);
            } else if (!autopilot && ekfHealthy) {
                momentsCmd =
                    inner.computeWrench(
                        outer.out.attCmd,
                        manPsi,
                        navState.segment<3>(0),
                        ctrl_filter.output().segment<3>(3) - imuStats.segment<3>(0),
                        acc_conInner);
            } else if (!autopilot && !ekfHealthy) {
                attManual(2) = AHRSAtt(2);
                momentsCmd =
                    inner.computeWrench(
                        attManual,
                        manPsi,
                        AHRSAtt,
                        ctrl_filter.output().segment<3>(3) - imuStats.segment<3>(0),
                        acc_conInner);

                double den = cos(attManual(0)) * cos(attManual(1));
                den = clamp(den, 0.2, 1.0);

                double Fz_base = mass * g * (1 - manVel(2));
                outer.out.Fz = clamp(Fz_base / den, 0, 2 * mass * g);
            } else { // Autopilot ON, EKF UNHEALTHY
                momentsCmd =
                    inner.computeWrench(
                        Vec<3>::Zero(),
                        0.0,
                        AHRSAtt,
                        ctrl_filter.output().segment<3>(3) - imuStats.segment<3>(0),
                        acc_conInner);

                double den = cos(AHRSAtt(0)) * cos(AHRSAtt(1));
                den = clamp(den, 0.2, 1.0);

                double Fz_base = mass * g * (1.0 - 0.5);
                outer.out.Fz = clamp(Fz_base / den, 0.0, 2.0 * mass * g);
            }

            wrenchCmd << outer.out.Fz,
                momentsCmd(0),
                momentsCmd(1),
                momentsCmd(2);

            thrustCmd = mixer.mix2Thrust(wrenchCmd);
            pwmCmd = mixer.thr2PWM(thrustCmd);
            throttleTest << rcPWM(2), rcPWM(2), rcPWM(2), rcPWM(2);

            acc_conInner = 0.0;

            // Real commands
            motor_task.updateState(pwmCmd, rcPWM(4));
        }

        // ---------------- Telemetry ----------------
        if (acc_tele >= rate_tele) {
            ts.t = t;
            ts.dt = dt;
            ts.Hz = Hz;
            ts.navState = navState;
            ts.posCmd = MM.out.posCmd;
            ts.phase = static_cast<int>(MM.out.phase);
            ts.mode = static_cast<int>(MM.out.mode);
            ts.attCmd = outer.out.attCmd;
            ts.armed = motor_task.isArmed();
            ts.NIS = NIS;
            ts.res = ekf.getRes();
            ts.PWMcmd = pwmCmd;
            telemetry_task.updateState(ts);
            acc_tele = 0.0;
        }

        // ---------------- Console Print (timing only) ----------------
        if (t - lastPrint >= printDt) {
            std::cout << "t=" << t
                      << "s  TargetHz=" << (1.0 / target_dt)
                      << "  Hz=" << Hz
                      << "  AvgSleep[us]=" << avg_sleep * 1e6
                      << "  AvgComp[us]=" << avg_compute * 1e6
                      << "  Overruns=" << overrun_count
                      << std::endl;
            lastPrint = t;
        }

        // --------- Loop timing: compute sleep at fixed target rate ---------
        auto loop_end = clock_t::now();
        std::chrono::duration<double> compute_dur = loop_end - loop_start;
        double compute_s = compute_dur.count();

        // EMA for compute
        if (avg_compute == 0.0) {
            avg_compute = compute_s;
        } else {
            avg_compute = (1.0 - ema_alpha) * avg_compute + ema_alpha * compute_s;
        }

        double sleep_s = target_dt - compute_s;
        if (sleep_s > 0.0) {
            // Track the intended sleep for logging
            if (avg_sleep == 0.0) {
                avg_sleep = sleep_s;
            } else {
                avg_sleep = (1.0 - ema_alpha) * avg_sleep + ema_alpha * sleep_s;
            }

            // HYBRID SLEEP
            // 1. usleep for the bulk of the time, stopping 100 microseconds early
            double early_wake_s = sleep_s - 0.000100; 
            if (early_wake_s > 0.0) {
                usleep(static_cast<int>(early_wake_s * 1e6));
            }

            // 2. Spin-lock (busy wait) for the final fractions of a millisecond
            while (true) {
                auto current_time = clock_t::now();
                std::chrono::duration<double> elapsed = current_time - loop_start;
                if (elapsed.count() >= target_dt) {
                    break; 
                }
            }
        } else {
            overrun_count++;
        }

        // No in-run tuning of target_dt: timing stats (avg_sleep, avg_compute, overrun_count)
        // are only for logging so you can adjust target_dt between runs.
    }

    // Unreachable in current structure, but keep for completeness
    motor_task.stop();
    if (motor_thread.joinable()) {
        motor_thread.join();
    }

    telemetry_task.stop();
    if (telemetry_thread.joinable()) {
        telemetry_thread.join();
    }

    return 0;
}

