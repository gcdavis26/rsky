#include <iostream>
#include <iomanip>
#include "common/TimeKeeper.h"
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
#include <unistd.h>
#include "mocap/mocapHandler.h"


int main() {

    std::cout << std::fixed << std::setprecision(4);
    //init objects
    TimeKeeper clock;
    ModeManager MM;
    OuterLoop outer;
    InnerLoop inner;
    QuadMixer mixer;
    UdpSender udp("192.168.1.2", 8080); //KINETIC 192.168.1.2
    TelemetryState ts;

    TelemetryTask telemetry_task(udp);
    std::thread telemetry_thread(&TelemetryTask::loop, &telemetry_task);

    RCIn rcin;
    rcin.initialize();

    MotorDriver motdrv;
    motdrv.initialize();

    MotorTask<MotorDriver> motor_task(motdrv);
    std::thread motor_thread(&MotorTask<MotorDriver>::loop, &motor_task);

    IMUHandler imuReal;
    Vec<12> imuStats = imuReal.initialize(); //(mgx,mgy,mgz,max,may,maz,siggx,siggy,siggz,sigax,sigay,sigaz)
    imuStats(5) = imuStats(5) + g;
    AHRS ahrs(imuStats.segment<6>(0));
    EKF ekf(imuStats.segment<6>(0)); // add bias constructor
    MocapHandler mocap;
    bool mocapInit = mocap.init();

    ImuLpf ekf_filter(952.0f, 80.0f);
    ImuLpf ctrl_filter(952.0f, 160.0f);

    //init vars
    double lastPrint = 0.0;
    const double printDt = 1.0;

    Vec<3> momentsCmd = Vec<3>::Zero();
    Vec<4> thrustCmd = Vec<4>::Zero();
    Vec<4> wrenchCmd = Vec<4>::Zero();
    Vec<4> pwmCmd = Vec<4>::Zero();
    Vec<6> rcPWM = Vec<6> ::Zero();
    Vec<4> throttleTest = Vec<4>::Zero();

    Vec<3> manVel = Vec<3>::Zero();
    double manPsi = 0.0;

    int step = 0;
    double Hz = 0.0;
    double HzTimer = 0.0;
    int HzCounter = 0;

    bool autopilot = false;
    bool printOn = true;

    double NIS = 4.0;
    bool ekfHealthy = false;
    double ekfbadTimer = 0.0;
    double ekfgoodTimer = 0.0;

    while (true) {

        double dt = clock.dt();
        double t = clock.elapsed();
        clock.stepClocks(dt);
        step++;

        HzTimer += dt;
        HzCounter++;

        if (HzTimer >= 0.1) {
            Hz = HzCounter / HzTimer;
            HzTimer = 0.0;
            HzCounter = 0;
        }

        if (clock.taskClock.imu >= clock.rates.imu) {
            imuReal.update();
            clock.taskClock.imu = 0.0;
            Vec<6> raw;
            raw.segment<3>(0) = imuReal.imu.accel;
            raw.segment<3>(3) = imuReal.imu.gyro;
            ekf_filter.update(raw);
            ctrl_filter.update(raw);
        }

        if (clock.taskClock.opti >= clock.rates.opti) {
            mocap.update();
            clock.taskClock.opti = 0.0;
        }

        // ---------------- EKF ----------------

        if (!ekf.init && mocapInit) {
            ekf.initializeFromOpti(mocap.opti);
            ekf.init = true;
        }

        if (clock.taskClock.navPred >= clock.rates.navPred) {
            ekf.predict(ekf_filter.output(), clock.taskClock.navPred);
            clock.taskClock.navPred = 0.0;
        }

        if ((clock.taskClock.navCorr >= clock.rates.navCorr) && mocap.m_valid) {
            ekf.correct(mocap.opti);

            NIS = ekf.getHealth();
            if (NIS > 13.28) {
                ekfbadTimer += clock.taskClock.navCorr;
                ekfgoodTimer = 0.0;
            }
            else {
                ekfbadTimer = 0.0;
                ekfgoodTimer += clock.taskClock.navCorr;
            }

            if (ekfbadTimer > 5.0) {
                ekfHealthy = false;
            }
            else if (ekfgoodTimer > 1.0) {
                ekfHealthy = true;
            }

            clock.taskClock.navCorr = 0.0;
        }
        Vec<15> navState = ekf.getx();

        // ---------------- Mode Manager ----------------
        if (clock.taskClock.MM >= clock.rates.MM) {
            MM.in.state = navState;
            MM.in.dt = dt;
            MM.in.detected = false;
            MM.update();
            clock.taskClock.MM = 0.0;
        }

        // ---------------- Manual RC Controls (Linux Only) -------------------
        if (clock.taskClock.keys >= clock.rates.keys) {
            Vec<3> rcVel = Vec<3>::Zero();
            double rcPsi = 0.0;

            rcPWM = rcin.read_ppm_vector();

            if (rcPWM(5) > 1750) {
                //drop stuff
            }
            else if (rcPWM(5) > 1250) {
                autopilot = true;
            }
            else {
                autopilot = false;
            }

            Vec<4> normalizedPWM = normPWM(rcPWM.segment<4>(0));

            rcVel(0) = normalizedPWM(1);
            rcVel(1) = normalizedPWM(0);
            rcVel(2) = -normalizedPWM(2);

            rcPsi = normalizedPWM(3);

            clock.taskClock.keys = 0.0;

            if (motor_task.isArmed()) {
                manPsi = rcPsi;
                manVel = rcVel; // 1m/s max speed in each direction 
            }
        }
            // ---------------- Outer Loop ----------------

            if (clock.taskClock.conOuter >= clock.rates.conOuter) {
                if (autopilot) {
                    outer.in.state = navState.segment<6>(3);
                    outer.in.posCmd = MM.out.posCmd;
                    outer.in.psi = navState(2);
                    outer.in.mode = MM.out.mode;
                    outer.update();
                }
                else {
                    outer.in.state = navState.segment<6>(3);
                    outer.in.posCmd = navState.segment<3>(3);
                    outer.in.psi = navState(2);
                    outer.in.mode = ModeManager::NavMode::Manual;
                    outer.in.velCmd = manVel;
                    outer.update();
                    outer.out.attCmd(2) = navState(2);
                }
                clock.taskClock.conOuter = 0.0;
            }

            // ---------------- AHRS ------------------------
            if (!ahrs.init) {
                ahrs.initializeFromAccel(ctrl_filter.output().segment<3>(0));
                ahrs.init = true;
            }
            if (clock.taskClock.AHRS >= clock.rates.AHRS) {
                ahrs.update(ctrl_filter.output().segment<3>(0), ctrl_filter.output().segment<3>(3), clock.taskClock.AHRS);
                clock.taskClock.AHRS = 0.0;
            }

            Vec<3> AHRSAtt = ahrs.euler();
            Vec<3> attManual;
            // ---------------- Inner Loop ---------------- %% CHANGE IMU TO IMUREAL IF YOU ARE DOING TESTING!!!!

            if (clock.taskClock.conInner >= clock.rates.conInner) {
                attManual << 10 * PI / 180 * manVel(1), -10 * PI / 180 * manVel(0), navState(2);
                manPsi = manPsi * 10 * PI / 180;

                autopilot = false;
                ekfHealthy = false;

                if (autopilot && ekfHealthy) {
                    momentsCmd =
                        inner.computeWrench(
                            outer.out.attCmd,
                            0.0,
                            navState.segment<3>(0),
                            ctrl_filter.output().segment<3>(3),
                            clock.taskClock.conInner);
                }
                else if (!autopilot && ekfHealthy) {
                    momentsCmd =
                        inner.computeWrench(
                            outer.out.attCmd,
                            manPsi,
                            navState.segment<3>(0),
                            ctrl_filter.output().segment<3>(3),
                            clock.taskClock.conInner);
                }
                else if (!autopilot && !ekfHealthy) {
                    attManual(2) = AHRSAtt(2);
                    momentsCmd =
                        inner.computeWrench(
                            attManual,
                            manPsi,
                            AHRSAtt,
                            ctrl_filter.output().segment<3>(3), clock.taskClock.conInner);

                    double den = cos(AHRSAtt(0)) * cos(AHRSAtt(1));
                    den = clamp(den, 0.2, 1.0);

                    double Fz_base = mass * g * (1 - manVel(2));
                    outer.out.Fz = clamp(Fz_base / den, 0, 2 * mass * g);
                }
                else {
                    momentsCmd =
                        inner.computeWrench(
                            Vec<3>::Zero(),
                            0.0,
                            AHRSAtt,
                            ctrl_filter.output().segment<3>(3),
                            clock.taskClock.conInner);
                    outer.out.Fz = mass * g;
                }

                wrenchCmd << outer.out.Fz,
                    momentsCmd(0),
                    momentsCmd(1),
                    momentsCmd(2);

                thrustCmd = mixer.mix2Thrust(wrenchCmd);

                pwmCmd = mixer.thr2PWM(thrustCmd); //this will go directly to the four motors
		throttleTest << rcPWM(2),rcPWM(2),rcPWM(2),rcPWM(2);
                clock.taskClock.conInner = 0.0;

                // ----------------Real Commands -------------
                motor_task.updateState(throttleTest, rcPWM(4));
            }

            // ---------------- Telemetry -----------------
            if (clock.taskClock.tele >= clock.rates.tele) {
		navState.segment<3>(0) = AHRSAtt;
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
                ts.PWMcmd = throttleTest;
                telemetry_task.updateState(ts);
                clock.taskClock.tele = 0.0;
            }

            // ---------------- Console Print ----------------
            if (t - lastPrint >= printDt && printOn) {

                // Clear screen + cursor home (no helpers)

                const Vec<3> pos = navState.segment<3>(3);
                const Vec<3> vel = navState.segment<3>(6);
                const Vec<3> rpy = navState.segment<3>(0);

                const Vec<3> posCmd = MM.out.posCmd;

                const Vec<3> attCmd = attManual;

                const Vec<3> gyro = ctrl_filter.output().segment<3>(3);
                const Vec<3> accel = ctrl_filter.output().segment<3>(0);

                const Vec<3> optPos = mocap.opti.pos;
                const double optPsi = mocap.opti.psi;

                std::cout
                    << "====================== QUADCOPTER STATE ======================\n"
                    << "  Time [s]: " << std::setw(8) << t
                    << " Rate [Hz]: " << std::setw(8) << Hz
                    << "      Mode: " << std::setw(8) << static_cast<int>(MM.out.mode)
                    << "     Armed: " << std::setw(8) << motor_task.isArmed() << "\n"
                    << "--------------------------------------------------------------\n"

                    << " NAV (EKF)\n"
                    << " EKF Health : "
                    << std::setw(8) << NIS << "\n"
                    << "   Pos [N E D] : "
                    << std::setw(8) << pos(0) << " "
                    << std::setw(8) << pos(1) << " "
                    << std::setw(8) << pos(2) << "\n"

                    << "   Vel [N E D] : "
                    << std::setw(8) << vel(0) << " "
                    << std::setw(8) << vel(1) << " "
                    << std::setw(8) << vel(2) << "\n"

                    << "  EKF Att [R P Y] : "
                    << std::setw(8) << rpy(0) << " "
                    << std::setw(8) << rpy(1) << " "
                    << std::setw(8) << rpy(2) << "\n\n"

                    << "  AHRS Att [R P Y] : "
                    << std::setw(8) << AHRSAtt(0) << " "
                    << std::setw(8) << AHRSAtt(1) << " "
                    << std::setw(8) << AHRSAtt(2) << "\n\n"

                    << " COMMANDS\n"
                    << "   PosCmd [N E D] : "
                    << std::setw(8) << posCmd(0) << " "
                    << std::setw(8) << posCmd(1) << " "
                    << std::setw(8) << posCmd(2) << "\n"

                    << "   VelCmd [N E D] : "
                    << std::setw(8) << outer.in.velCmd(0) << " "
                    << std::setw(8) << outer.in.velCmd(1) << " "
                    << std::setw(8) << outer.in.velCmd(2) << "\n"

                    << "   AttCmd [R P Y] : "
                    << std::setw(8) << attCmd(0) << " "
                    << std::setw(8) << attCmd(1) << " "
                    << std::setw(8) << attCmd(2) << "\n"

                    << "   FzCmd          : "
                    << std::setw(8) << outer.out.Fz << "\n"

                    << "   PWMCmd         : "
                    << std::setw(8) << pwmCmd(0) << " "
                    << std::setw(8) << pwmCmd(1) << " "
                    << std::setw(8) << pwmCmd(2) << " "
                    << std::setw(8) << pwmCmd(3) << "\n"

                    << "   rcPWM          : "
                    << std::setw(8) << rcPWM(0) << " "
                    << std::setw(8) << rcPWM(1) << " "
                    << std::setw(8) << rcPWM(2) << " "
                    << std::setw(8) << rcPWM(3) << " "
                    << std::setw(8) << rcPWM(4) << " "
                    << std::setw(8) << rcPWM(5) << "\n\n"

                    << " SENSORS\n"
                    << "   IMU Stats : "
                    << std::setw(8) << imuStats.transpose() << "\n"
                    << "   IMU Gyro  [x y z] : "
                    << std::setw(8) << gyro(0) << " "
                    << std::setw(8) << gyro(1) << " "
                    << std::setw(8) << gyro(2) << "\n"

                    << "   IMU Accel [x y z] : "
                    << std::setw(8) << accel(0) << " "
                    << std::setw(8) << accel(1) << " "
                    << std::setw(8) << accel(2) << "\n"

                    << "   Opti Pos  [N E D] : "
                    << std::setw(8) << optPos(0) << " "
                    << std::setw(8) << optPos(1) << " "
                    << std::setw(8) << optPos(2) << "\n"

                    << "   Opti Psi          : "
                    << std::setw(8) << optPsi << "\n\n"

                    << " ACTUATION\n"
                    << "   WrenchCmd [Fz Mx My Mz] : "
                    << std::setw(8) << wrenchCmd(0) << " "
                    << std::setw(8) << wrenchCmd(1) << " "
                    << std::setw(8) << wrenchCmd(2) << " "
                    << std::setw(8) << wrenchCmd(3) << "\n"

                    << "   ThrustCmd [t1 t2 t3 t4] : "
                    << std::setw(8) << thrustCmd(0) << " "
                    << std::setw(8) << thrustCmd(1) << " "
                    << std::setw(8) << thrustCmd(2) << " "
                    << std::setw(8) << thrustCmd(3) << "\n"

                    << "==============================================================\n";

                lastPrint = t;
            }

#ifdef PLATFORM_LINUX
            //usleep(1);
#endif
    //std::this_thread::yield();
    }

    motor_task.stop();
    if (motor_thread.joinable()) {
        motor_thread.join();
    }

    telemetry_task.stop();
    if (telemetry_thread.joinable()) {
        telemetry_thread.join();
    }
}

