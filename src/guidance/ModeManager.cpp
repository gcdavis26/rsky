#include "guidance/ModeManager.h"

void ModeManager::update() {
	double surveyAlt = -2.0;
	double lowAlt = -0.5;

	if (!init) {
		init = true;
		toCmd(0) = in.state(3);
		toCmd(1) = in.state(4);
		toCmd(2) = surveyAlt;
	}

	out.phaseTime += in.dt;

	switch (out.phase) {
		case MissionPhase::Takeoff:
			out.mode = NavMode::Waypoint;
			out.posCmd = toCmd;

			if (reachedWaypoint(in.state.segment<6>(3), out.posCmd)) {
				advancePhase(MissionPhase::Search);
			}
			break;
		case MissionPhase::Search: 
			out.mode = NavMode::Sweep;
			out.posCmd = in.state.segment<3>(3);
			if (in.detected) {
				advancePhase(MissionPhase::GoToTarget);
			}
			break;
		case MissionPhase::GoToTarget:
			out.mode = NavMode::Waypoint;
			out.posCmd.segment<2>(0) = in.targPos.segment<2>(0);
			out.posCmd(2) = surveyAlt;
			if (reachedWaypoint(in.state.segment<6>(3), out.posCmd)) {
				advancePhase(MissionPhase::Hover);
			}
			break;
		case MissionPhase::Hover:
			out.mode = NavMode::Waypoint;
			out.posCmd.segment<2>(0) = in.targPos.segment<2>(0);
			out.posCmd(2) = surveyAlt;
			if (reachedWaypoint(in.state.segment<6>(3), out.posCmd) && out.phaseTime >= 5.0) {
				advancePhase(MissionPhase::DescendToTarget);
			}
			break;
		case MissionPhase::DescendToTarget:
			out.mode = NavMode::Waypoint;
			out.posCmd.segment<2>(0) = in.targPos.segment<2>(0);
			out.posCmd(2) = lowAlt;
			if (reachedWaypoint(in.state.segment<6>(3),out.posCmd) && in.drop) {
				computelndCmd(in.state.segment<3>(3));
			}
			break;
		case MissionPhase::GoToLand:
			out.mode = NavMode::Waypoint;
			out.posCmd.segment<2>(0) = lndCmd.segment<2>(0);
			out.posCmd(2) = lowAlt;
			if (reachedWaypoint(in.state.segment<6>(3), out.posCmd) && out.phaseTime >= 5.0) {
				advancePhase(MissionPhase::DescendToLand);
			}
			break;
		case MissionPhase::DescendToLand:
			out.mode = NavMode::Waypoint;
			out.posCmd.segment<2>(0) = lndCmd.segment<2>(0);
			out.posCmd(2) = 0.0;
			if (reachedWaypoint(in.state.segment<6>(3), out.posCmd) && out.phaseTime >= 5.0) {
				advancePhase(MissionPhase::Terminate);
			}
			break;
		case MissionPhase::Terminate:
			out.mode = NavMode::Waypoint;
			out.posCmd = in.state.segment<3>(3);
			break;
	}
}

bool ModeManager::reachedWaypoint(const Vec<6>&state,const Vec<3>& cmd) {
	double posErr = (cmd - state.segment<3>(0)).norm();
	double velErr = state.segment<3>(3).norm();

	return (posErr < posTol) && (velErr < velTol);
}

void ModeManager::advancePhase(MissionPhase next) {
	out.phase = next;
	out.phaseTime = 0.0;
}

void ModeManager::computelndCmd(const Vec<3>& pos) {
	Vec<2> center;
	Vec<2> dir;
	center << cn, ce;
	
	double dist = (center - pos.segment<2>(0)).norm();

	if (dist < 1e-6) {
		lndCmd.segment<2>(0) = pos.segment<2>(0);
		return;
	}
	
	dir = (center - pos.segment<2>(0)).normalized();

	lndCmd.segment<2>(0) = pos.segment<2>(0) + clearDist * dir;
}
