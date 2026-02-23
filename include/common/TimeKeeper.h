#pragma once
#include <chrono>

class TimeKeeper {
public:
	TimeKeeper();
	void reset();
	double dt();
	double elapsed() const;
	void stepClocks(double dt);

	struct scheduler {
		static constexpr double navPred = 1.0 / 100;
		static constexpr double navCorr = 1.0 / 25;
		static constexpr double conInner = 1.0 / 400;
		static constexpr double conOuter = 1.0 / 50;
		static constexpr double tele = 1.0 / 10;
		static constexpr double imu = 1.0 / 100;
		static constexpr double opti = 1.0 / 25;
		static constexpr double MM = 1.0 / 50;
		static constexpr double AHRS = 1.0 / 100;
	};

	struct accumulator {
		double navPred = 0.0;
		double navCorr = 0.0;
		double conInner = 0.0;
		double conOuter = 0.0;
		double tele = 0.0;
		double imu = 0.0;
		double opti = 0.0;
		double MM = 0.0;
		double AHRS = 0.0;
	};

	scheduler rates;
	accumulator taskClock;

private:
	using Clock = std::chrono::steady_clock;

	Clock::time_point t_start;
	Clock::time_point t_prev;
	Clock::time_point t_now;

};
