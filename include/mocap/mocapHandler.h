#pragma once

#include "common/MathUtils.h"

class MocapHandler {
public:

	MocapHandler();

	int init();

	int update();

	Vec<4> getMeas() const;

	Vec<3> getPosition() const;

	Eigen::Quaternionf getQuaternion() const;

	struct OptiMeas {
		Vec<3> pos = Vec<3>::Zero();
		double psi = 0.0;
	};
	OptiMeas opti;

private:

	Vec<4> m_ned;
	Eigen::Quaternionf m_quaternion;
	int m_valid;
	int m_frameNum;

	float quaternionToHeading(const Eigen::Quaternionf& q) const;

};