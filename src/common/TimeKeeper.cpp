#include "common/TimeKeeper.h"

TimeKeeper::TimeKeeper() {
	reset();
}

void TimeKeeper::reset() {
	t_start = Clock::now();
	t_prev = t_start;
	t_now = t_start;
}

double TimeKeeper::dt() {
	t_now = Clock::now();
	double dt = std::chrono::duration<double>(t_now - t_prev).count();
	t_prev = t_now;
	return dt;
}

double TimeKeeper::elapsed() const {
	return std::chrono::duration<double>(Clock::now() - t_start).count();
}

void TimeKeeper::stepClocks(double dt) {
	taskClock.navPred += dt;
	taskClock.navCorr += dt;
	taskClock.conInner += dt;
	taskClock.conOuter += dt;
	taskClock.tele += dt;
	taskClock.imu += dt;
	taskClock.opti += dt;
}