#include "control/InnerLoop.h"

Vec<3> InnerLoop::computeWrench(const Vec<3>& att_cmd, double yaw_rate_cmd, const Vec<3>& att, const Vec<3>& omega) {

	Vec<3> attErr = wrapAngles(att_cmd - att); 

	Vec<3> wrench;

	wrench = kp.cwiseProduct(attErr) - kd.cwiseProduct(omega);
	wrench(2) = kd(2)*(yaw_rate_cmd - omega(2));

	wrench(0) = clamp(wrench(0), -Mx_max, Mx_max);
	wrench(1) = clamp(wrench(1), -My_max, My_max);
	wrench(2) = clamp(wrench(2), -Mz_max, Mz_max);

	return wrench;

}