#include "mocap/mocapHandler.h"
#include "mocap/network.h"

static Vec<3> optitrackToNED(double ot_x, double ot_y, double ot_z) {
	return Vec<3>(-ot_z, ot_x, -ot_y);
}

MocapHandler::MocapHandler() :
	m_ned(Vec<4>::Zero()),
	m_quaternion(Eigen::Quaternionf::Identity()),
	m_valid(0),
	m_frameNum(0)
{
}

int MocapHandler::init() {
	int result = openPort();
	if (result != 0) {
		printf("Failed to init Mocap Port");
	}
	else {
		printf("Socket open, waiting for data");
	}
	return result;
}

int MocapHandler::update() {
	int gotPacket = readDatalink();

	if (gotPacket) {
		m_quaternion = Eigen::Quaternionf(onboardMocapClient.qw, onboardMocapClient.qx, onboardMocapClient.qy, onboardMocapClient.qz).normalized();
		Vec<3> pos = optitrackToNED(onboardMocapClient.pos_x, onboardMocapClient.pos_y, onboardMocapClient.pos_z);
		m_ned(0) = quaternionToHeading(m_quaternion);
		m_ned(1) = pos(0); 
		m_ned(2) = pos(1);
		m_ned(3) = pos(2);

		opti.pos = pos;
		opti.psi = m_ned(0);

		m_valid = onboardMocapClient.valid;
		m_frameNum = onboardMocapClient.frameNum;
	}

	return gotPacket;
}

float MocapHandler::quaternionToHeading(const Eigen::Quaternionf& q) const {
	float sinY = 2.0f * (q.w() * q.y() + q.z() * q.x());
	float cosY = 1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y());
	return atan2f(sinY, cosY);
}

Vec<4>   MocapHandler::getMeas()        const { return m_ned; }
Vec<3>    MocapHandler::getPosition()   const { return m_ned.tail<3>(); }
Eigen::Quaternionf MocapHandler::getQuaternion() const { return m_quaternion; }