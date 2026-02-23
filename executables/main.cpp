#include <iostream>
#include <iomanip>

#ifdef _WIN32
    #include <WinSock2.h>
    #include <Windows.h>
    #include <thread>
#endif

#include "common/TimeKeeper.h"
#include "common/MathUtils.h"
#include "control/InnerLoop.h"
#include "control/OuterLoop.h"
#include "control/QuadMixer.h"
#include "estimation/EKF.h"
#include "guidance/ModeManager.h"
#include "sensors/ImuSim.h"
#include "sensors/OptiSim.h"
#include "simulator/Dynamics.h"
#include "simulator/MotorModel.h"
#include "telemetry/udp_sender.h"
#include "estimation/AHRS.h"

#ifdef PLATFORM_LINUX
    #include "drivers/MotorDriver.h"
    #include "drivers/RCIn.h"
    #include "sensors/IMUHandler.h"
    #include <unistd.h>
#endif

int main() {

    std::cout << std::fixed << std::setprecision(4);

    TimeKeeper clock;
    Dynamics dynamics;
    ImuSim imu;
    OptiSim opti;
    EKF ekf;
    AHRS ahrs;
    ModeManager MM;
    OuterLoop outer;
    InnerLoop inner;
    QuadMixer mixer;
    MotorModel motormodel;
    UdpSender udp("127.0.0.1", 8080);
    
#ifdef PLATFORM_LINUX
    RCIn rcin;
    rcin.initialize();

    MotorDriver motdrv;

    IMUHandler imuReal;
#endif

    double lastPrint = 0.0;
    const double printDt = 0.1; 

    Vec<4> thrustCmd = Vec<4>::Zero();
    Vec<4> wrenchCmd = Vec<4>::Zero();
    Vec<4> pwmCmd = Vec<4>::Zero();
    Vec<6> rcPWM = Vec<6> ::Zero();

    Vec<4> thrustAct = Vec<4>::Zero();
    Vec<4> wrenchAct = Vec<4>::Zero();

    int step = 0;
    double Hz = 0.0;
    double HzTimer = 0.0;
    int HzCounter = 0;

    bool autopilot = true;
    bool printOn = false;
    bool armed = true;
    bool motorInit = false;

    double NIS = 4.0;

    while (true) {

        const double dt = clock.dt();
        const double t = clock.elapsed();
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
            imu.step(dynamics.getTrueState(), clock.taskClock.imu);
#ifdef PLATFORM_LINUX
            imuReal.update();
#endif
            clock.taskClock.imu = 0.0;
        }

        if (clock.taskClock.opti >= clock.rates.opti) {
            opti.step(dynamics.getTrueState());
            clock.taskClock.opti = 0.0;
        }

        // ---------------- EKF ----------------
        if (!ekf.init) {
            ekf.initializeFromOpti(opti.opti);
            ekf.init = true;
        }

        if (clock.taskClock.navPred >= clock.rates.navPred) {
            ekf.predict(imu.imu, clock.taskClock.navPred);
            clock.taskClock.navPred = 0.0;
        }

        if (clock.taskClock.navCorr >= clock.rates.navCorr) {
            ekf.correct(opti.opti);
            NIS = ekf.getHealth();
            clock.taskClock.navCorr = 0.0;
        }
        const Vec<15> navState = ekf.getx();

        // ---------------- Mode Manager ----------------
        if (clock.taskClock.MM >= clock.rates.MM) {
            MM.in.state = navState;
            MM.in.dt = dt;
            MM.update();
            clock.taskClock.MM = 0.0;
        }

        // Manual Command Types
        Vec<3> manVel = Vec<3>::Zero();
        double manPsi = 0.0;

        // ---------------- Manual Keyboard Controls (Windows Only) -----------
#ifdef _WIN32
        if (clock.taskClock.conInner >= clock.rates.conInner) {
            Vec<3> keyVel = Vec<3>::Zero();
            double keyPsi = 0.0;

            if (GetAsyncKeyState('W') & 0x8000) {
                keyVel(0) = 1;
            }
            else if (GetAsyncKeyState('S') & 0x8000) {
                keyVel(0) = -1;
            }

            if (GetAsyncKeyState('A') & 0x8000) {
                keyVel(1) = -1;
            }
            else if (GetAsyncKeyState('D') & 0x8000) {
                keyVel(1) = 1;
            }

            if (GetAsyncKeyState(VK_SPACE) & 0x8000) {
                keyVel(2) = -1;
            }
            else if (GetAsyncKeyState(VK_SHIFT) & 0x8000) {
                keyVel(2) = 1;
            }

            if (GetAsyncKeyState('Q') & 0x8000) {
                keyPsi = -1;
            }
            else if (GetAsyncKeyState('E') & 0x8000) {
                keyPsi = 1;
            }
            if (armed) {
                manVel = keyVel;
                manPsi = keyPsi;
            }
        }
#endif

        // ---------------- Manual RC Controls (Linux Only) -------------------
#ifdef PLATFORM_LINUX
        if (clock.taskClock.conInner >= clock.rates.conInner) {
            Vec<3> rcVel = Vec<3>::Zero();
            double rcPsi = 0.0;

            rcPWM = rcin.read_ppm_vector();

            if (rcPWM(4) > 1500.0) {
                armed = true;
            }
            else {
                armed = false;
            }

            if (rcPWM(5) > 1750) {
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

            rcPsi = 5 * PI / 180 * normalizedPWM(3);

            if (armed) {
                manPsi = rcPsi;
                manVel = rcVel; // 1m/s max speed in each direction 
            }
        }
#endif
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
            ahrs.initializeFromAccel(imu.imu.accel);
            ahrs.init = true;
        }
        if (clock.taskClock.AHRS >= clock.rates.AHRS) {
            ahrs.update(imu.imu.accel, imu.imu.gyro, clock.taskClock.AHRS);
            clock.taskClock.AHRS = 0.0;
        }
        Vec<3> AHRSAtt = ahrs.euler();

        // ---------------- Inner Loop ----------------
        if (clock.taskClock.conInner >= clock.rates.conInner) {
            Vec<3> attManual;
            attManual << 10 * PI / 180 * manVel(1), -10 * PI / 180 * manVel(0), 0.0;

            const Vec<3> momentsCmd =
                inner.computeWrench(
                    outer.out.attCmd,
                    0.0,
                    navState.segment<3>(0),
                    imu.imu.gyro);

            wrenchCmd << outer.out.Fz,
                momentsCmd(0),
                momentsCmd(1),
                momentsCmd(2);

            thrustCmd = mixer.mix2Thrust(wrenchCmd);

            if (!armed) {
                thrustCmd = Vec<4>::Zero();
            }
            pwmCmd = mixer.thr2PWM(thrustCmd);
            clock.taskClock.conInner = 0.0;
        }

        // ----------------Real Commands -------------
#ifdef PLATFORM_LINUX
        /*
    if (!motorInit && armed) {
        motdrv.initialize();
        motorInit = true;
    }
    else if (motorInit && armed) {
        Vec<4> pwmOut = Vec<4>::Zero();
    }
    else {
        motdrv.wind_down();
        motorInit = false;
    }
        */
#endif
        // ---------------- Telemetry -----------------
        if (clock.taskClock.tele >= clock.rates.tele) {
            udp.sendFromSim(
                t, dt, Hz,
                navState,
                MM,
                outer,
                imu,
                armed
            );
            clock.taskClock.tele = 0.0;
        }
        // ---------------- Simulation ----------------
        if (clock.taskClock.sim >= clock.rates.sim) {
            thrustAct = motormodel.step(clock.taskClock.sim, thrustCmd);
            wrenchAct = mixer.mix2Wrench(thrustAct);

            dynamics.step(clock.taskClock.sim, wrenchAct);

            clock.taskClock.sim = 0.0;

        }

        // ---------------- Console Print ----------------
        if (t - lastPrint >= printDt && printOn) {

            // Clear screen + cursor home (no helpers)
            std::cout << "\033[2J\033[H";

            const Vec<3> pos = navState.segment<3>(3);
            const Vec<3> vel = navState.segment<3>(6);
            const Vec<3> rpy = navState.segment<3>(0);

            const Vec<3> posCmd = MM.out.posCmd;
  
            const Vec<3> attCmd = outer.out.attCmd;

#ifdef _WIN32
            const Vec<3> gyro = imu.imu.gyro;
            const Vec<3> accel = imu.imu.accel;
#endif

#ifdef PLATFORM_LINUX
            const Vec<3> gyro = imuReal.imu.gyro;
            const Vec<3> accel = imuReal.imu.accel;
#endif
            const Vec<3> optPos = opti.opti.pos;
            const double optPsi = opti.opti.psi;

            std::cout
                << "====================== QUADCOPTER STATE ======================\n"
                << "  Time [s]: " << std::setw(8) << t
                << " Rate [Hz]: " << std::setw(8) << Hz
                << "      Mode: " << std::setw(8) << static_cast<int>(MM.out.mode)
                    << "     Armed: " << std::setw(8) << armed << "\n"
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
#ifdef PLATFORM_LINUX
                    << "   rcPWM          : "
                    << std::setw(8) << rcPWM(0) << " "
                    << std::setw(8) << rcPWM(1) << " "
                    << std::setw(8) << rcPWM(2) << " "
                    << std::setw(8) << rcPWM(3) << " "
                    << std::setw(8) << rcPWM(4) << " "
                    << std::setw(8) << rcPWM(5) << "\n\n"
#endif

                    << " SENSORS\n"
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

                    << "   ThrustAct [t1 t2 t3 t4] : "
                    << std::setw(8) << thrustAct(0) << " "
                    << std::setw(8) << thrustAct(1) << " "
                    << std::setw(8) << thrustAct(2) << " "
                    << std::setw(8) << thrustAct(3) << "\n"

                    << "   WrenchAct [Fz Mx My Mz] : "
                    << std::setw(8) << wrenchAct(0) << " "
                    << std::setw(8) << wrenchAct(1) << " "
                    << std::setw(8) << wrenchAct(2) << " "
                    << std::setw(8) << wrenchAct(3) << "\n"

                    << "==============================================================\n";
                

            lastPrint = t;
        }
        
#ifdef PLATFORM_LINUX
	//usleep(1);
#endif
    //std::this_thread::sleep_for(std::chrono::microseconds(1));
    std::this_thread::yield();
    }
}
