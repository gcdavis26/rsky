#include "mocap/mocapHandler.h"
#include "mocap/network.h"
#include "common/MathUtils.h"

static Vec<3> optitrackToNED(double ot_x, double ot_y, double ot_z) {
	return Vec<3>(-ot_y+9.144/2,-ot_x+4.572/2,-ot_z);
}

MocapHandler::MocapHandler() :
	m_ned(Vec<4>::Zero()),
	m_quaternion(Eigen::Quaternionf::Identity()),
	m_valid(0),
	m_frameNum(0)
{
	q_prev = Vec<4>::Zero();
	q_prev << 1.0,0.0,0.0,0.0;
}

int MocapHandler::init() {
	bool result = openPort();
	if (result != 0) {
		printf("Failed to init Mocap Port");
	}
	else {
		printf("Socket open, waiting for data");
	}
	return !result;
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
		opti.psi =-m_ned(0);

		m_valid = onboardMocapClient.valid;
		m_frameNum = onboardMocapClient.frameNum;
	}

	return gotPacket;
}

float MocapHandler::quaternionToHeading(const Eigen::Quaternionf& q) {
	double w,x,y,z;
	double dot = q.w()*q_prev(0)+q.x()*q_prev(1)+q.y()*q_prev(2)+q.z()*q_prev(3);
	if (dot<0.0) {
	   w = -q.w();
	   x = -q.x();
    	   y = -q.y();
	   z = -q.z();
	}
	else {
 	  w = q.w();
	  x = q.x();
	  y = q.y();
	  z = q.z();
	}

	q_prev << w,x,y,z;

	float yaw = std::atan2(
		2.0*(w*z+x*z),
		1.0 - 2.0 * (y*y+z*z)
	);
	return yaw;
}

Vec<4>   MocapHandler::getMeas()        const { return m_ned; }
Vec<3>    MocapHandler::getPosition()   const { return m_ned.tail<3>(); }
Eigen::Quaternionf MocapHandler::getQuaternion() const { return m_quaternion; }
