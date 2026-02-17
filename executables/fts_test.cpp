#include <cstdio>
#include <unistd.h>
#include <memory>
#include "Common/Util.h"
#include "RCInput.h"
#include "MotorDriver.h"
#include <iostream>

constexpr int INPUT_CHANNEL = 2;      // RC input channel to read (throttle)
constexpr int OUTPUT_CHANNEL = 0;     // PWM output channel to control motor
constexpr int PWM_FREQUENCY = 50;     // 50 Hz typical for servo/ESC
constexpr int FEED_US = 20000;        // 20 ms update interval (50 Hz)

int main()
{
	MotorDriver motordriver;
	RCInputHandler controller;

	motordriver.initialize();

	while (true)
	{
		Eigen::Matrix<double, 6,1> ppm = controller.read_ppm_vector();
		std::cout << ppm(2) << "|" << ppm(5) << std::endl;

		Eigen::Matrix<double, 4, 1> motoppm = Eigen::Matrix<double, 4, 1>::Zero();
		motoppm << ppm(2), ppm(2), ppm(2), ppm(2);

		motordriver.command(motoppm);

		if (static_cast<int>(ppm(5)) == 2000)
		{
			motordriver.~MotorDriver();
			break;
		}
	}
}



