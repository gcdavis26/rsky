#pragma once

#include <Eigen/Dense>

#include "common/types.h"   // Vec3
#include "plant/quad_state.h"

namespace gnc {

// ============================
// Parameters
// ============================
struct ModeManagerParams {
  double survey_height_m = -2.0;     // meters above ground
  double low_height_m    = -0.5;     // meters above ground

  double move_to_center_dist_m = 1.5;

  double tol_xy_m = 0.15;
  double tol_z_m  = 0.10;

  double hover_at_target_s = 5.0;
};

// ============================
// Navigation / Mission enums
// ============================
enum class NavMode {
  Waypoint,
  Guidance
};

enum class MissionPhase {
  TakeoffToSurvey = 0,
  SweepGuidance,
  GoToTargetAtSurveyAlt,
  HoverAtTarget,
  DescendToLow,
  Action,
  MoveTowardCenterAtLow,
  LandStraightDown,
  Done
};

// ============================
// Inputs from other modules
// ============================
struct ModeManagerInputs {
  bool detected = false;
  Vec3 target_ned = Vec3::Zero();
  bool action_done = false;
  Vec3 active_center_ned = Vec3::Zero();
};

// ============================
// Outputs to controllers
// ============================
struct ModeManagerOutput {
  NavMode nav_mode = NavMode::Waypoint;
  MissionPhase phase = MissionPhase::TakeoffToSurvey;

  Vec3 pos_cmd_ned = Vec3::Zero();
  double yaw_cmd_rad = 0.0;

  bool latched_detection = false;
};

// ============================
// Mode Manager class
// ============================
class ModeManager {
public:
  explicit ModeManager(const ModeManagerParams& params);

  void reset();

  ModeManagerOutput update(const QuadStateTruth& state,
                           const ModeManagerInputs& inputs,
                           double dt);

  MissionPhase phase() const;
  bool latchedDetection() const;

private:
  // Parameters
  ModeManagerParams params_;

  // Internal state
  bool initialized_;
  MissionPhase phase_;
  double phase_time_s_;

  bool latched_detection_;

  Eigen::Vector2d takeoff_hold_ne_;
  double yaw_hold_rad_;

  Eigen::Vector2d cached_target_ne_;
  Eigen::Vector2d cached_center_ne_;
  Eigen::Vector2d center_step_target_ne_;

private:
  void advancePhase(MissionPhase next);

  bool reachedWaypoint(const Vec3& p_now,
                       const Vec3& p_cmd) const;

  void computeCenterStepTarget(const Vec3& p_now_ned);
};

} // namespace gnc
