#include "control/OuterLoop.h"

void OuterLoop::update() {

    Vec<3> posErr = in.posCmd - in.state.segment<3>(0);
    Vec<3> velErr = -in.state.segment<3>(3);

    Kp << kpn, kpe, kpd;
    Kd << kdn, kde, kdd;
    Ki_pos << kin, kie, kid;
    Ki_vel << kivn, kive, kivd;
    Ki_sweep << ki_sweep_n, ki_sweep_e, ki_sweep_d;

    // ---- mode transition detection ----
    bool modeChanged = (in.mode != prevMode);
    prevMode = in.mode;

    Vec<3> accCmd;

    if (in.mode == ModeManager::NavMode::Waypoint) {

        if (in.arm) {
            if (modeChanged) {
                // FIX: Only back-solve if we are already in the air (Z pos < -0.2m)
                // Otherwise, start the integrator at zero to avoid clamping downwards on takeoff
                if (in.state(2) < -0.2) { 
                    posInt = (accCmd_prev - Kp.cwiseProduct(posErr)
                        - Kd.cwiseProduct(velErr))
                        .cwiseQuotient(Ki_pos.cwiseMax(1e-9));
                    posInt = posInt.cwiseMax(-posIntMax).cwiseMin(posIntMax);
                } else {
                    posInt = Vec<3>::Zero();
                }
            }
            posInt += posErr * in.dt;
            posInt = posInt.cwiseMax(-posIntMax).cwiseMin(posIntMax);
        }
        
        else {
            posInt = Vec<3>::Zero();
        }

        accCmd = Kp.cwiseProduct(posErr)
            + Kd.cwiseProduct(velErr)
            + Ki_pos.cwiseProduct(posInt);

        // zero other integrators
        velInt = Vec<3>::Zero();
        sweepInt = Vec<3>::Zero();
    }
    else if (in.mode == ModeManager::NavMode::Manual) {

        Vec<3> velErrManual = in.velCmd + velErr; // velCmd - velocity

        if (in.arm) {
            if (modeChanged) {
                velInt = (accCmd_prev - Kd.cwiseProduct(velErrManual))
                    .cwiseQuotient(Ki_vel.cwiseMax(1e-9));
                velInt = velInt.cwiseMax(-velIntMax).cwiseMin(velIntMax);
            }
            velInt += velErrManual * in.dt;
            velInt = velInt.cwiseMax(-velIntMax).cwiseMin(velIntMax);
        }
        else {
            velInt = Vec<3>::Zero();
        }

        accCmd = Kd.cwiseProduct(velErrManual)
            + Ki_vel.cwiseProduct(velInt);

        // zero other integrators
        posInt = Vec<3>::Zero();
        sweepInt = Vec<3>::Zero();
    }
    else {
        // sweep mode
        Vec<3> sweepErr;
        Vec<3> sweepPD = sweepControl(sweepErr);

        if (in.arm) {
            if (modeChanged) {
                sweepInt = (accCmd_prev - sweepPD)
                    .cwiseQuotient(Ki_sweep.cwiseMax(1e-9));
                sweepInt = sweepInt.cwiseMax(-sweepIntMax).cwiseMin(sweepIntMax);
            }
            sweepInt += sweepErr * in.dt;
            sweepInt = sweepInt.cwiseMax(-sweepIntMax).cwiseMin(sweepIntMax);
        }
        else {
            sweepInt = Vec<3>::Zero();
        }

        accCmd = sweepPD + Ki_sweep.cwiseProduct(sweepInt);

        // zero other integrators
        posInt = Vec<3>::Zero();
        velInt = Vec<3>::Zero();
    }

    // save for bumpless transfer on next mode switch
    accCmd_prev = accCmd;

    // ---- attitude command ----
    Mat<2, 2> A;
    A << -sin(in.psi), cos(in.psi),
        -cos(in.psi), -sin(in.psi);

    out.attCmd.segment<2>(0) = (1.0 / g) * A * accCmd.segment<2>(0);
    out.attCmd(2) = in.psi;
    out.attCmd(0) = clamp(out.attCmd(0), -maxAtt, maxAtt);
    out.attCmd(1) = clamp(out.attCmd(1), -maxAtt, maxAtt);

    double den = cos(in.phi) * cos(in.theta);
    den = clamp(den, 0.2, 1.0);
    double FzCmd = mass * (g - accCmd(2)) / den;
    out.Fz = clamp(FzCmd, Fz_min, Fz_max);
}

Vec<3> OuterLoop::sweepControl(Vec<3>& sweepErr) {
    Vec<3> pos = in.state.segment<3>(0);
    Vec<3> vel = in.state.segment<3>(3);

    if (!sweep.init) {
	sweep.numStr = std::max(2,sweep.numStr);
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

    // velocity error for N/E, position error for D
    Vec<2> velErrNE = vCmd - vel.segment<2>(0);
    double posErrD = surveyAlt - pos(2);

    // output error vector for integrator (accumulated in update())
    sweepErr << velErrNE(0), velErrNE(1), posErrD;

    // PD-only acceleration command
    Vec<3> accCmd;
    accCmd << kVel * velErrNE(0),
        kVel* velErrNE(1),
        kpd* posErrD - kdd * vel(2);

    return accCmd;
}
