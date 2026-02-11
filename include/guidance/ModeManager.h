#pragma once

#include "common/MathUtils.h"

class ModeManager {
public:
	enum class NavMode {
		Manual,
		Waypoint,
		Sweep
	};

	enum class MissionPhase {
		Takeoff,
		Search,
		GoToTarget,
		Hover,
		DescendToTarget,
		GoToLand,
		DescendToLand,
		Terminate
	};

	struct input {
		bool detected = false;
		Vec<15> state = Vec<15>::Zero();
		Vec<3> targPos = Vec<3>::Zero();
		bool drop = false;
		double dt;
	};

	input in;

	struct output {
		NavMode mode = NavMode::Manual;
		MissionPhase phase = MissionPhase::Takeoff;
		Vec<3> posCmd = Vec<3>::Zero();
		double yaw_cmd_rad = 0.0;
		double phaseTime = 0.0;
	};

	output out;

	void update();

private:

	bool init = false;
	
	bool reachedWaypoint(const Vec<6>& state,const Vec<3>& cmd);

	void advancePhase(MissionPhase next);

	void computelndCmd(const Vec<3>& p_now_ned);

	double cn = 9.144 / 2;
	double ce = 4.572 / 2; 

	double clearDist = 1.5;

	double velTol = 0.05;
	double posTol = 0.05;

	Vec<3> toCmd;
	Vec<3> lndCmd;

};