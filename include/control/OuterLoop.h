#pragma once
#include "common/MathUtils.h"
#include "guidance/ModeManager.h"

class OuterLoop {
public:
	struct input {
		Vec<6> state = Vec<6>::Zero();
		Vec<3> posCmd = Vec<3>::Zero();
		Vec<3> velCmd = Vec<3>::Zero();
		double phi = 0.0;
		double theta = 0.0;
		double psi = 0.0;
		ModeManager::NavMode mode;
		double dt = 0.0;
		bool arm = false;
	};
	input in;

	struct output {
		Vec<3> attCmd = Vec<3>::Zero();
		double Fz = 0.0;
	};
	output out;

	void update();
	//getter functions

	Vec<3> getAccels()
	{
		return accCmd_prev;
	}
	Vec<3> getIError()
	{
		return posInt;
	}

private:

	Vec<3> Kp;
	Vec<3> Kd;
	Vec<3> Ki_pos;
	Vec<3> Ki_vel;
	Vec<3> Ki_sweep;

	Vec<3> posInt = Vec<3>::Zero();
	Vec<3> velInt = Vec<3>::Zero();
	Vec<3> sweepInt = Vec<3>::Zero();

	// mode transition / bumpless transfer state
	ModeManager::NavMode prevMode = ModeManager::NavMode::Manual;
	Vec<3> accCmd_prev = Vec<3>::Zero();

	// integrator clamp bounds (per-axis)
	Vec<3> posIntMax = (Vec<3>() << 5.0, 5.0, 25.0).finished();
	Vec<3> velIntMax = (Vec<3>() << 5.0, 5.0, 10.0).finished();
	Vec<3> sweepIntMax = (Vec<3>() << 5.0, 5.0, 10.0).finished();

	struct SweepState {
		int stripeIdx = 1;
		int dir = 1;
		int numStr = 1;
		int pass = 1;
		bool init = false;
	};
	SweepState sweep;

	double buffer = 1.5;
	double n_min = 0 + buffer;
	double n_max = 9.144 - buffer;
	double e_min = 0 + buffer;
	double e_max = 4.572 - buffer;
	double n_margin = 0.50;
	double deStripe = 0.0;

	double v_sweep = 1.0;
	double kp_cross = 3;
	double kd_cross = 1.25;
	double kVel = 1.25;
	double surveyAlt = -2.0;

	double ki_sweep_n = 0.01;
	double ki_sweep_e = 0.01;
	double ki_sweep_d = 0.25;

	double maxAtt = 10 * PI / 180;
	double Fz_min = 0.0;
	double Fz_max = 4.0 * mass * g;

	double kpn = 4;
	double kpe = 4;

	double kpd = 3.75;

	double kdn = 1.25;
	double kde = 1.25;
	double kdd = 3.5;

	double kin = 0.01;
	double kie = 0.01;
	double kid = 0.4;

	double kivn = 0.1;
	double kive = 0.1;
	double kivd = 0.025;

	// returns PD-only accCmd, outputs error vector for integrator
	Vec<3> sweepControl(Vec<3>& sweepErr);


};
