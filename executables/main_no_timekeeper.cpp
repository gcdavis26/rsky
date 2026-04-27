#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <functional>

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
#include "vision/WildfireDetection.h"

#include <fstream>



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
    ModeManager MM(true);
    OuterLoop outer;
    InnerLoop inner;
    QuadMixer mixer;
    UdpSender udp("192.168.1.13", 8080); // KINETIC 192.168.1.2
    TelemetryState ts;
    StateBuffer shared_state;
    HotspotBuffer shared_targets;
    VisionGridBuffer vision_buffer;

    TelemetryTask telemetry_task(udp, vision_buffer);
    std::thread telemetry_thread(&TelemetryTask::loop, &telemetry_task);

    RCIn rcin;
    rcin.initialize();
    MotorDriver motdrv;
    //motdrv.calibrate(); 
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
    bool opti_new = mocap.update();

    ImuLpf ekf_filter(600.0f, 250.0f);
    ImuLpf ctrl_filter(600.0f, 250.0f);
    ekf_filter.on = true;
    ctrl_filter.on = true;

    Vec<3> momentsCmd = Vec<3>::Zero();
    Vec<4> thrustCmd = Vec<4>::Zero();
    Vec<4> wrenchCmd = Vec<4>::Zero();
    Vec<4> pwmCmd = Vec<4>::Zero();
    Vec<6> rcPWM = Vec<6> ::Zero();
    Vec<4> throttleTest = Vec<4>::Zero();
    Vec<6> raw;
    double dropper = 1000;
    Vec<3> manVel = Vec<3>::Zero();
    double manPsi = 0.0;

    double Hz = 0.0;

    bool autopilot = false;

    double NIS = 4.0;
    bool ekfHealthy = false;
    double ekfbadTimer = 0.0;
    double ekfgoodTimer = 0.0;

    // --- New timing model (no TimeKeeper): fixed loop period per run ---
    using clock_t = std::chrono::steady_clock;
    auto start_time = clock_t::now();
    auto last_time = start_time;

    // Fixed target period (change this value to adjust control rate between runs)
    double target_dt = 1.0 / 600.0;     // 650 Hz target loop rate

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
    const double rate_imu = 1.0 / 600.0;
    const double rate_opti = 1.0 / 120.0;
    const double rate_navPred = 1.0 / 600.0;
    const double rate_navCorr = 1.0 / 120.0;
    const double rate_MM = 1.0 / 100.0;
    const double rate_keys = 1.0 / 100.0;
    const double rate_conOuter = 1.0 / 200.0;
    const double rate_AHRS = 1.0 / 600.0;
    const double rate_conInner = 1.0 / 600.0;
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

    std::ofstream logger("datalog.csv");
    logger << "n,e,d,an,ae,ad\n";

/* camr
    std::thread vision_thread(
        wildfireDetectionTask, 
        std::ref(shared_state), 
        std::ref(shared_targets), 
        std::ref(vision_buffer)
    );
*/

    while (true) {
        auto loop_start = clock_t::now();
        std::chrono::duration<double> dt_ch = loop_start - last_time;
        double dt = dt_ch.count();
        last_time = loop_start;

        t = std::chrono::duration<double>(loop_start - start_time).count();

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
            opti_new = mocap.update();
            acc_opti -= rate_opti;
        }


        // ---------------- EKF ----------------
        if (!ekf.init && mocapInit && opti_new) {
            ekf.initializeFromOpti(mocap.opti);
            ekf.init = true;
        }

        if ((acc_navPred >= rate_navPred) && ekf.init) {
            ekf.predict(ekf_filter.output(), acc_navPred);
            acc_navPred = 0.0;
        }

        if ((acc_navCorr >= rate_navCorr) && mocap.m_valid && ekf.init && opti_new) {
            ekf.correct(mocap.opti);
	    opti_new = false;

            NIS = ekf.getHealth();
            if (NIS > 13.28) {
                ekfbadTimer += acc_navCorr;
                ekfgoodTimer = 0.0;
            } else {
                ekfbadTimer = 0.0;
                ekfgoodTimer += acc_navCorr;
            }

            if (ekfbadTimer > 10.0) {
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
            MM.in.dt = acc_MM;
            MM.in.detected = false;
            MM.update();
            acc_MM = 0.0;
        }

        // ---------------- Manual RC Controls ----------------
        if (acc_keys >= rate_keys) {
            Vec<3> rcVel = Vec<3>::Zero();
            double rcPsi = 0.0;

            rcPWM = rcin.read_ppm_vector();
	    if (MM.out.phase  == ModeManager::MissionPhase::Terminate)
	    {
		rcPWM(4) = 1000;
	    }

            if (rcPWM(5) > 1750 && motor_task.isArmed()) {
                dropper = 2000;
                MM.in.drop = true;
		autopilot = false;
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
                if(!logger.is_open())
                {
                    logger.open("datalog.csv");
                }
            }
            else
            {
                if (logger.is_open())
                {
                    logger.close();
                }
            }
        }

        // ---------------- Outer Loop ----------------
        if (acc_conOuter >= rate_conOuter) {
            if (autopilot) {
                outer.in.state = navState.segment<6>(3);
                outer.in.posCmd = MM.out.posCmd;
		        outer.in.phi = navState(0);
		        outer.in.theta = navState(1);
                outer.in.psi = navState(2);
                outer.in.mode = MM.out.mode;
                outer.in.dt = acc_conOuter;
                outer.in.arm = motor_task.isArmed();
                outer.update();
            } else {
                outer.in.state = navState.segment<6>(3);
                outer.in.posCmd = navState.segment<3>(3);
		        outer.in.phi = navState(0);
		        outer.in.theta = navState(1);
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

            Eigen::Matrix<double, 6, 1> vision_state;

            if (autopilot && ekfHealthy) {
		outer.out.attCmd(2) = 0.0;
                momentsCmd =
                    inner.computeWrench(
                        outer.out.attCmd,
                        0.0,
                        navState.segment<3>(0),
                        ctrl_filter.output().segment<3>(3) - imuStats.segment<3>(0),
                        acc_conInner);

                // Drone is using EKF, pass EKF attitude and position
                //vision_state = navState.head<6>(); camr

            } else if (!autopilot && ekfHealthy) {
                momentsCmd =
                    inner.computeWrench(
                        outer.out.attCmd,
                        manPsi,
                        navState.segment<3>(0),
                        ctrl_filter.output().segment<3>(3) - imuStats.segment<3>(0),
                        acc_conInner);

                // Drone is using EKF, pass EKF attitude and position
                //vision_state = navState.head<6>(); camr

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

                // Drone is using AHRS, pass AHRS attitude and EKF position
                //vision_state << AHRSAtt(0), AHRSAtt(1), AHRSAtt(2), camr
                                //navState(3), navState(4), navState(5); camr

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

                // Drone is using AHRS, pass AHRS attitude and EKF position
                //vision_state << AHRSAtt(0), AHRSAtt(1), AHRSAtt(2), camr
                                //navState(3), navState(4), navState(5); camr
            }

            //shared_state.update(vision_state); camr

            wrenchCmd << outer.out.Fz,
                momentsCmd(0),
                momentsCmd(1),
                momentsCmd(2);

            thrustCmd = mixer.mix2Thrust(wrenchCmd);
            pwmCmd = mixer.thr2PWM(thrustCmd);
            throttleTest << rcPWM(2), rcPWM(2), rcPWM(2), rcPWM(2);

            acc_conInner = 0.0;

            // Real commands
            motor_task.updateState(pwmCmd, rcPWM(4),dropper);
        }
        //----------------- Printing commands for testing ------------------
        // Vec<3> ierror = outer.getIError();
        // Vec<3> accels = outer.getAccels();
        //logger << ierror(0) << "," << ierror(1) << "," << ierror(2) << "," << accels(0) << "," << accels(1) << "," << accels(2) << "\n";

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

        // --------- Loop timing: compute sleep at fixed target rate ---------
        auto loop_end = clock_t::now();
        std::chrono::duration<double> compute_dur = loop_end - loop_start;
        double compute_s = compute_dur.count();


        double sleep_s = target_dt - compute_s;
        if (sleep_s > 0.0) {
            // usleep for the bulk of the time, stopping 100 microseconds early
            double early_wake_s = sleep_s - 0.000100; 
            if (early_wake_s > 0.0) {
                usleep(static_cast<int>(early_wake_s * 1e6));
            }

            // Spin-lock (busy wait) for the final fractions of a millisecond
            while (true) {
                auto current_time = clock_t::now();
                std::chrono::duration<double> elapsed = current_time - loop_start;
                if (elapsed.count() >= target_dt) {
                    break; 
                }
            }
        }

    }

    motor_task.stop();
    if (motor_thread.joinable()) {
        motor_thread.join();
    }

    telemetry_task.stop();
    if (telemetry_thread.joinable()) {
        telemetry_thread.join();
    }

    //vision_thread.detach(); 

    return 0;
}

