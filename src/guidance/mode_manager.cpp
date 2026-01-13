#include "guidance/mode_manager.h"

#include <cmath>

namespace gnc {

// ============================
// Constructor
// ============================
ModeManager::ModeManager(const ModeManagerParams& params)
  : params_(params) {
  reset();
}

// ============================
// Reset
// ============================
void ModeManager::reset() {
  initialized_ = false;
  phase_ = MissionPhase::TakeoffToSurvey;
  phase_time_s_ = 0.0;

  latched_detection_ = false;

  takeoff_hold_ne_.setZero();
  yaw_hold_rad_ = 0.0;

  cached_target_ne_.setZero();
  cached_center_ne_.setZero();
  center_step_target_ne_.setZero();
}

// ============================
// Public getters
// ============================
MissionPhase ModeManager::phase() const {
  return phase_;
}

bool ModeManager::latchedDetection() const {
  return latched_detection_;
}

// ============================
// Main update
// ============================
ModeManagerOutput ModeManager::update(const QuadStateTruth& state,
                                      const ModeManagerInputs& inputs,
                                      double dt) {
  ModeManagerOutput out;

  // Initialize on first call
  if (!initialized_) {
    initialized_ = true;

    takeoff_hold_ne_(0) = state.pos(0);
    takeoff_hold_ne_(1) = state.pos(1);
    yaw_hold_rad_ = state.euler(2);
  }

  phase_time_s_ += dt;

  // Latch detection
  if (inputs.detected) {
    latched_detection_ = true;
    cached_target_ne_(0) = inputs.target_ned(0);
    cached_target_ne_(1) = inputs.target_ned(1);
  }

  cached_center_ne_(0) = inputs.active_center_ned(0);
  cached_center_ne_(1) = inputs.active_center_ned(1);

  out.phase = phase_;
  out.yaw_cmd_rad = yaw_hold_rad_;
  out.latched_detection = latched_detection_;

  const double d_survey = params_.survey_height_m;
  const double d_low    = params_.low_height_m;

  const Vec3 p_now = state.pos;

  switch (phase_) {

    case MissionPhase::TakeoffToSurvey:
      out.nav_mode = NavMode::Waypoint;
      out.pos_cmd_ned << takeoff_hold_ne_(0),
                          takeoff_hold_ne_(1),
                          d_survey;

      if (reachedWaypoint(p_now, out.pos_cmd_ned)) {
        advancePhase(MissionPhase::SweepGuidance);
      }
      break;

    case MissionPhase::SweepGuidance:
      out.nav_mode = NavMode::Guidance;

      if (latched_detection_) {
        advancePhase(MissionPhase::GoToTargetAtSurveyAlt);
      }
      break;

    case MissionPhase::GoToTargetAtSurveyAlt:
      out.nav_mode = NavMode::Waypoint;
      out.pos_cmd_ned << cached_target_ne_(0),
                          cached_target_ne_(1),
                          d_survey;

      if (reachedWaypoint(p_now, out.pos_cmd_ned)) {
        advancePhase(MissionPhase::HoverAtTarget);
      }
      break;

    case MissionPhase::HoverAtTarget:
      out.nav_mode = NavMode::Waypoint;
      out.pos_cmd_ned << cached_target_ne_(0),
                          cached_target_ne_(1),
                          d_survey;

      if (phase_time_s_ >= params_.hover_at_target_s) {
        advancePhase(MissionPhase::DescendToLow);
      }
      break;

    case MissionPhase::DescendToLow:
      out.nav_mode = NavMode::Waypoint;
      out.pos_cmd_ned << cached_target_ne_(0),
                          cached_target_ne_(1),
                          d_low;

      if (reachedWaypoint(p_now, out.pos_cmd_ned)) {
        advancePhase(MissionPhase::Action);
      }
      break;

    case MissionPhase::Action:
      out.nav_mode = NavMode::Waypoint;
      out.pos_cmd_ned << cached_target_ne_(0),
                          cached_target_ne_(1),
                          d_low;

      if (inputs.action_done) {
        computeCenterStepTarget(state.pos);
        advancePhase(MissionPhase::MoveTowardCenterAtLow);
      }
      break;

    case MissionPhase::MoveTowardCenterAtLow:
      out.nav_mode = NavMode::Waypoint;
      out.pos_cmd_ned << center_step_target_ne_(0),
                          center_step_target_ne_(1),
                          d_low;

      if (reachedWaypoint(p_now, out.pos_cmd_ned) && state.vel.norm() <= 0.01 ) {
        advancePhase(MissionPhase::LandStraightDown);
      }
      break;

    case MissionPhase::LandStraightDown:
      out.nav_mode = NavMode::Waypoint;
      out.pos_cmd_ned << center_step_target_ne_(0),
                          center_step_target_ne_(1),
                          0.0;

      if (std::abs(p_now(2)) < params_.tol_z_m &&
          std::abs(state.vel(2)) < 0.25) {
        advancePhase(MissionPhase::Done);
      }
      break;

    case MissionPhase::Done:
      out.nav_mode = NavMode::Waypoint;
      out.pos_cmd_ned << center_step_target_ne_(0),
                          center_step_target_ne_(1),
                          0.0;
      break;
  }

  out.phase = phase_;
  out.latched_detection = latched_detection_;
  return out;
}

// ============================
// Helpers
// ============================
void ModeManager::advancePhase(MissionPhase next) {
  phase_ = next;
  phase_time_s_ = 0.0;
}

bool ModeManager::reachedWaypoint(const Vec3& p_now,
                                  const Vec3& p_cmd) const {
  const double dn = p_now(0) - p_cmd(0);
  const double de = p_now(1) - p_cmd(1);
  const double dd = p_now(2) - p_cmd(2);

  const double horiz = std::sqrt(dn * dn + de * de);
  const double vert  = std::abs(dd);

  return (horiz < params_.tol_xy_m) &&
         (vert  < params_.tol_z_m);
}

void ModeManager::computeCenterStepTarget(const Vec3& p_now_ned) {
  const double n_now = p_now_ned(0);
  const double e_now = p_now_ned(1);

  double vn = cached_center_ne_(0) - n_now;
  double ve = cached_center_ne_(1) - e_now;

  const double norm = std::sqrt(vn * vn + ve * ve);

  if (norm < 1e-6) {
    center_step_target_ne_(0) = n_now;
    center_step_target_ne_(1) = e_now;
    return;
  }

  vn /= norm;
  ve /= norm;

  center_step_target_ne_(0) =
      n_now + params_.move_to_center_dist_m * vn;
  center_step_target_ne_(1) =
      e_now + params_.move_to_center_dist_m * ve;
}

} // namespace gnc
