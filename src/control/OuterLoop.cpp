#include "control/OuterLoop.h"

void OuterLoop::update() {

		Vec<3> posErr;
		posErr = in.posCmd - in.state.segment<3>(0);

		Vec<3> velErr;
		velErr = -in.state.segment<3>(3);

		Kp << kpn, kpe, kpd;
		Kd << kdn, kde, kdd;

		Vec<3> accCmd;

		if (in.mode == ModeManager::NavMode::Waypoint) {
			accCmd = Kp.cwiseProduct(posErr) + Kd.cwiseProduct(velErr);
		}
        else if (in.mode == ModeManager::NavMode::Manual) {
            accCmd = Kd.cwiseProduct(in.velCmd + velErr);
        }
		else {
			accCmd = sweepControl();
		}

        Mat<2, 2> A;
        A << -sin(in.psi), cos(in.psi),
            -cos(in.psi), -sin(in.psi);

        out.attCmd.segment<2>(0) = 1 / g * A * accCmd.segment<2>(0);
        out.attCmd(2) = 0.0;

        out.attCmd(0) = clamp(out.attCmd(0), -maxAtt, maxAtt);
        out.attCmd(1) = clamp(out.attCmd(1), -maxAtt, maxAtt);

        double den = cos(out.attCmd(0)) * cos(out.attCmd(1));
        den = clamp(den, 0.2, den);
        double FzCmd = mass * (g - accCmd(2)) / den;
        out.Fz = clamp(FzCmd, Fz_min, Fz_max);
}

Vec<3> OuterLoop::sweepControl() {
	Vec<3> pos;
	Vec<3> vel;

	pos = in.state.segment<3>(0);
	vel = in.state.segment<3>(3);

	if (!sweep.init) {
		double widthE = e_max - e_min;
		deStripe = widthE / (sweep.numStr - 1);

		sweep.stripeIdx = 1;
		sweep.dir = 1;
		sweep.pass = 1;
		sweep.init = true;
	}
    bool at_top = (pos(0) >= (n_max - n_margin));
    bool at_bottom = (pos(0) <= (n_min + n_margin));

    if (sweep.pass == 1) {
        if ((sweep.dir > 0) && at_top) {
            if (sweep.stripeIdx < sweep.numStr) {
                sweep.dir = -1;
                sweep.stripeIdx = sweep.stripeIdx + 1;
            }
            else {
                sweep.pass = 2;
                sweep.dir = -1;
                sweep.stripeIdx = sweep.stripeIdx - 1;
            }
        }
        else if ((sweep.dir < 0) && at_bottom) {
            if (sweep.stripeIdx < sweep.numStr) {
                sweep.dir = +1;
                sweep.stripeIdx = sweep.stripeIdx + 1;
            }
            else {
                sweep.pass = 2;
                sweep.dir = +1;
                sweep.stripeIdx = sweep.stripeIdx - 1;
            }
        }
    }
    else {
        if ((sweep.dir > 0) && at_top) {
            if (sweep.stripeIdx > 1) {
                sweep.dir = -1;
                sweep.stripeIdx = sweep.stripeIdx - 1;
            }
            else {
                sweep.pass = 1;
                sweep.dir = -1;
                sweep.stripeIdx = sweep.stripeIdx + 1;
            }
        }
        else if ((sweep.dir < 0) && at_bottom) {
            if (sweep.stripeIdx > 1) {
                sweep.dir = +1;
                sweep.stripeIdx = sweep.stripeIdx - 1;
            }
            else {
                sweep.pass = 1;
                sweep.dir = 1;
                sweep.stripeIdx = sweep.stripeIdx + 1;
            }
        }
    }

    if (sweep.stripeIdx < 1) sweep.stripeIdx = 1;
    if (sweep.stripeIdx > sweep.numStr) sweep.stripeIdx = sweep.numStr;

    double eStripe = e_min + (sweep.stripeIdx - 1) * deStripe;
    double vnCmd = sweep.dir * v_sweep;
    double veCmd = kp_cross * (eStripe - pos(1)) - kd_cross * vel(1);

    Vec<2> vCmd;
    vCmd << vnCmd, veCmd;

    double speed = vCmd.norm();
    if (speed > 1e-9) {
        vCmd = vCmd * (v_sweep / speed);
    }
    else {
        vCmd << sweep.dir * v_sweep, 0.0;
    }

    Vec<2> accCmdne;
    accCmdne = kVel * (vCmd - vel.segment<2>(0));
    double accCmdd = kpd * (surveyAlt - pos(2)) - kdd * vel(2);
    
    Vec<3> accCmd;
    accCmd << accCmdne(0), accCmdne(1), accCmdd;
    return accCmd;
}