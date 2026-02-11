#pragma once
#include "common/MathUtils.h"
#include "guidance/ModeManager.h"

class OuterLoop {
public:
	struct input {
		Vec<6> state = Vec<6>::Zero();
		Vec<3> posCmd = Vec<3>::Zero();
		Vec<3> velCmd = Vec<3>::Zero();
		Vec<3> velCm = Vec<3>::Zero();
		double psi = 0.0;
		ModeManager::NavMode mode;
	};

	input in;

	struct output {
		Vec<3> attCmd = Vec<3>::Zero();
		double Fz = 0.0;
	};

	output out;

	void update();

private:

	Vec<3> Kp;
	Vec<3> Kd;

	struct SweepState {
		int stripeIdx = 1;
		int dir = 1;
		int numStr = 4;
		int pass = 1;
		bool init = false;
	};

	SweepState sweep;

	double buffer = 0.25;
	double n_min = 0 + buffer;
	double n_max = 9.144 - buffer;
	double e_min = 0 + buffer;
	double e_max = 4.572 - buffer;
	double n_margin = 0.50;
	double deStripe = 0.0;
	
	double v_sweep = 1.0;
	double kp_cross = 1.0;
	double kd_cross = 1.5;
	double kVel = 1.0;

	double surveyAlt = -2.0;

	double maxAtt = 10 * PI / 180;
	double Fz_min = 0.0;
	double Fz_max = 2 * mass * g;

	double kpn = 0.5;
	double kpe = 0.5;
	double kpd = 2.0;

	double kdn = 1.5;
	double kde = 1.5;
	double kdd = 2.0;

	Vec<3> sweepControl();
};