#include <iostream>
#include <cmath>

// Logging
#include "sim/logger.h"

// Sensors
#include "sensors/imu_sim.h"
#include "sensors/optitrack_sim.h"

// Estimation
#include "estimation/ekf15_full.h"

// Plant / dynamics
#include "plant/quad_params.h"
#include "plant/quad_dynamics_euler.h"
#include "plant/motor_model_first_order.h"
#include "plant/plant_types.h"

// Inner-loop attitude
#include "control/inner_loop_attitude_pd.h"
#include "control/attitude_pd_params.h"

// Waypoint outer-loop
#include "control/outer_loop_position_pd.h"
#include "control/position_pd_params.h"

// Sweep outer-loop (coverage)
#include "control/sweep_types.h"
#include "control/outer_loop_coverage.h"

// Mixer
#include "control/mixer_params.h"
#include "control/quad_mixer_x.h"

// Mode manager
#include "guidance/mode_manager.h"

// Common
#include "common/types.h"

int main() {

  // =========================
  // Logger
  // =========================
  gnc::Logger logger("sim_log.csv");
  if (!logger.isOpen()) {
    std::cout << "ERROR: could not open sim_log.csv\n";
    return 1;
  }

  // =========================
  // Quad parameters
  // =========================
  gnc::QuadParams quad;

  // =========================
  // Sensors
  // =========================
  gnc::ImuSimParams imu_p;
  gnc::ImuSim imu_sim(quad, imu_p);

  gnc::OptiSimParams opti_p;
  gnc::OptiTrackSim opti_sim(opti_p);

  // =========================
  // EKF
  // =========================

  gnc::Ekf15Full ekf;      // <-- if your class name differs, rename this line
  bool ekf_inited = false;

  // 10 Hz Opti update (dt=0.01 -> every 10 steps)
  const int opti_div = 1;
  int opti_counter = 0;

  // =========================
  // Dynamics
  // =========================
  gnc::QuadDynamics dynamics(quad);

  // =========================
  // Mixer parameters (X config)
  // =========================
  gnc::MixerParams mixer_params;
  gnc::QuadMixerX mixer(mixer_params);

  // =========================
  // Motor model (first-order lag)
  // =========================
  double motor_tau = 0.05; // 50 ms motor time constant
  gnc::MotorModelFirstOrder motor(motor_tau);

  // =========================
  // Initial state (NED)
  // =========================
  gnc::QuadStateTruth state;
  state.pos << 2, 2, 0;

  // =========================
  // Simulation settings
  // =========================
  const double dt = 0.01;    // 100 Hz
  int step = 0;

  // Termination logic settings
  const double ground_d_eps_m = 0.02;        // "on ground" if d <= this (NED down)
  const double still_speed_eps_mps = 0.05;   // "hasn't moved" if speed <= this
  const double still_time_required_s = 10.0; // must be still for this long

  // =========================
  // Controllers
  // =========================
  gnc::AttitudePDParams att_p;
  gnc::InnerLoopAttitudePD att_ctrl(att_p);

  gnc::PositionPDParams pos_p;
  gnc::OuterLoopPositionPD outer_waypoint(pos_p, quad);

  gnc::SweepParams sweep_p;
  gnc::OuterLoopCoverage outer_sweep(quad, sweep_p);
  gnc::SweepState sweepState;

  // =========================
  // Mode Manager
  // =========================
  gnc::ModeManagerParams mm_p;
  gnc::ModeManager mode_mgr(mm_p);

  // =========================
  // "Other code" placeholders
  // =========================
  bool detected_flag = false;
  bool action_done_flag = false;

  gnc::Vec3 target_ned;
  target_ned << 6.0, 3.0, 0.0;

  gnc::Vec3 active_center_ned;
  active_center_ned << 9.144/2, 4.572/2, 0.0;

  // Outputs used every loop
  gnc::Vec3 euler_cmd;
  euler_cmd.setZero();

  gnc::BodyWrench wrench_cmd;
  gnc::BodyWrench w_mom;
  gnc::BodyWrench wrench_act;

  gnc::Vec4 T_cmd;
  gnc::Vec4 T_act;
  gnc::Vec4 bodyWrench;

  gnc::Vec3 a_des;
  a_des.setZero();

  // =========================
  // Termination state
  // =========================
  double still_time_s = 0.0;

  // =========================
  // Main simulation loop
  // =========================
  while (true) {

    // -------------------------------------------------------
    // FAKE detection/action signals (TEMP for testing)
    // -------------------------------------------------------
    if (state.t > 30.0) detected_flag = true;
    if (state.t > 45.0) action_done_flag = true;

    // -------------------------------------------------------
    // Sensor simulation (from truth)
    // -------------------------------------------------------
    gnc::ImuMeas imu_meas  = imu_sim.step(state, dt);
    gnc::OptiMeas opti_meas = opti_sim.step(state);

    // -------------------------------------------------------
    // EKF init + predict + update
    // -------------------------------------------------------
	if (step % 1 == 0) {
		if (!ekf_inited) {
		  ekf.initializeFromOpti(opti_meas);
		  ekf_inited = true;
		}
		
		opti_counter++;
		if (opti_counter >= opti_div) {
		  opti_counter = 0;
		  ekf.updateOpti(opti_meas);
		}

		ekf.predict(imu_meas, 1 * dt);
		
	}
    // Build "estimates" in the same shapes your controllers expect
    const auto& xhat = ekf.x();
	


    gnc::Vec3 est_pos, est_vel, est_euler, est_omega_b;

    // Indices must match your EKF header. If your enum differs, change these.
    est_euler << xhat(gnc::Ekf15Full::PHI), xhat(gnc::Ekf15Full::THETA), xhat(gnc::Ekf15Full::PSI);
    est_pos   << xhat(gnc::Ekf15Full::PN),  xhat(gnc::Ekf15Full::PE),    xhat(gnc::Ekf15Full::PD);
    est_vel   << xhat(gnc::Ekf15Full::VN),  xhat(gnc::Ekf15Full::VE),    xhat(gnc::Ekf15Full::VD);

    // For inner loop, you can use gyro measurement as omega_b (typical)
    est_omega_b = imu_meas.gyro;

	const gnc::Vec3& nav_pos = est_pos;
	const gnc::Vec3& nav_vel = est_vel;
	const gnc::Vec3& nav_euler = est_euler;

	const gnc::Vec3& att_euler = est_euler; // inner loop does not like est maybe a tune?
    const gnc::Vec3& att_omega = est_omega_b;

    // -------------------------------------------------------
    // Mode manager (use nav_* if toggled)
    // -------------------------------------------------------
    gnc::QuadStateTruth nav_state = state;
    nav_state.pos   = est_pos;
    nav_state.vel   = est_vel;
    nav_state.euler = est_euler;

    gnc::ModeManagerInputs mm_in;
    mm_in.detected = detected_flag;
    mm_in.target_ned = target_ned;
    mm_in.action_done = action_done_flag;
    mm_in.active_center_ned = active_center_ned;

    gnc::ModeManagerOutput mm_out = mode_mgr.update(nav_state, mm_in, dt);

    // -------------------------------------------------------
    // Outer loop selection (Waypoint vs Sweep)
    // -------------------------------------------------------
		if (mm_out.nav_mode == gnc::NavMode::Waypoint) {

		  const gnc::Vec3 pos_cmd = mm_out.pos_cmd_ned;

		  outer_waypoint.compute(pos_cmd,
								 nav_pos,
								 nav_vel,
								 nav_euler,
								 euler_cmd,
								 wrench_cmd);

		  euler_cmd(2) = mm_out.yaw_cmd_rad;

		} else {

		  outer_sweep.compute(nav_pos,
							  nav_vel,
							  euler_cmd(2),
							  sweepState,
							  euler_cmd,
							  wrench_cmd,
							  a_des);
		}
    // -------------------------------------------------------
    // Inner loop attitude
    // -------------------------------------------------------
    w_mom = att_ctrl.compute(euler_cmd, att_euler, att_omega);

    wrench_cmd.Mx = w_mom.Mx;
    wrench_cmd.My = w_mom.My;
    wrench_cmd.Mz = w_mom.Mz;

    // -------------------------------------------------------
    // Mixer -> Motor lag -> Achieved wrench
    // -------------------------------------------------------
    T_cmd = mixer.mixToThrusts(wrench_cmd);
    T_cmd = mixer.clampThrusts(T_cmd);

    T_act = motor.step(dt, T_cmd);

    wrench_act = mixer.thrustsToWrench(T_act);

    bodyWrench << wrench_act.Fz,
                  wrench_act.Mx,
                  wrench_act.My,
                  wrench_act.Mz;

    // -------------------------------------------------------
    // Logging
    // -------------------------------------------------------
    logger.log(state,xhat,T_cmd);

    // Optional debug print once per second
    static double next_print_t = 0.0;
    if (state.t >= next_print_t) {

      std::cout
        << "t=" << state.t
        << " phase=" << static_cast<int>(mm_out.phase)
        << " nav=" << static_cast<int>(mm_out.nav_mode)
        << "\n";

      std::cout
        << "  TRUTH pos(n,e,d)=(" << state.pos(0) << "," << state.pos(1) << "," << state.pos(2) << ")"
        << " euler(phi,th,psi)=(" << state.euler(0) << "," << state.euler(1) << "," << state.euler(2) << ")"
		<< " vel(n,e,d)=(" << state.vel(0) << "," << state.vel(1) << "," << state.vel(2) << ")"
        << "\n";

      std::cout
        << "  EKF   pos(n,e,d)=(" << est_pos(0) << "," << est_pos(1) << "," << est_pos(2) << ")"
        << " euler(phi,th,psi)=(" << est_euler(0) << "," << est_euler(1) << "," << est_euler(2) << ")"
		<< " vel(n,e,d)=(" << est_vel(0) << "," << est_vel(1) << "," << est_vel(2) << ")"
        << "\n";

      std::cout
        << "  IMU: gyro(p,q,r)=(" << imu_meas.gyro(0) << "," << imu_meas.gyro(1) << "," << imu_meas.gyro(2) << ")"
        << " accel(ax,ay,az)=(" << imu_meas.accel(0) << "," << imu_meas.accel(1) << "," << imu_meas.accel(2) << ")"
        << "\n";

      std::cout
        << "  OPTI: pos(n,e,d)=(" << opti_meas.pos_ned(0) << "," << opti_meas.pos_ned(1) << "," << opti_meas.pos_ned(2) << ")"
        << " psi=" << opti_meas.psi
        << "\n";

      next_print_t += 1.0;
    }
	
	// -------------------------------------------------------
    // Plant dynamics (always truth)
    // -------------------------------------------------------
    dynamics.step(state, dt, bodyWrench);
	step++;

    // -------------------------------------------------------
    // Termination condition
    // -------------------------------------------------------
    const bool on_ground = (state.pos(2) >= -ground_d_eps_m);
    const double speed_mps = state.vel.norm();
    const bool not_moving = (speed_mps <= still_speed_eps_mps);

    if (on_ground && not_moving) still_time_s += dt;
    else still_time_s = 0.0;

    if (still_time_s >= still_time_required_s) {
      std::cout << "Termination: on ground and still for "
                << still_time_s << " s at t=" << state.t << "\n";
      break;
    }
  }

  // =========================
  // Final report
  // =========================
  std::cout << "Simulation complete.\n";
  std::cout << "Final position NED [m]: " << state.pos.transpose() << "\n";
  std::cout << "Final Euler angles [rad]: " << state.euler.transpose() << "\n";

  return 0;
}
