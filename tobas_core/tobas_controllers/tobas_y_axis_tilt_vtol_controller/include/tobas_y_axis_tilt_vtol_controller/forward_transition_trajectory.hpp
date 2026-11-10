#pragma once

#include <cmath>

#include <builtin_interfaces/msg/time.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
{
// TODO : 高速で固定翼で飛行している状態からの減速にも対応, 加速度拘束も考慮
class ForwardTransitionTrajectory
{
  static constexpr double kForwardAngle = 0.5 * M_PI; // [rad]
public:
  explicit ForwardTransitionTrajectory();
  void setMaxTiltVelocity(const double& max_tilt_velocity); // [rad/s]
  void setMinAoA(const double& min_alpha);
  void setMaxPitchRate(const double& max_pitch_rate); // [rad/s]
  void setCruiseVelocity(const double& cruise_speed);
  void setStartConditions(const builtin_interfaces::msg::Time& start_time, const double& start_pitch_angle, const double& start_tilt_angle, const double& start_vx);
  double calcTargetTiltAngle(const builtin_interfaces::msg::Time& cur_time);
  double calcTargetVx(const builtin_interfaces::msg::Time& cur_time);
  double calcTargetAccx(const builtin_interfaces::msg::Time& cur_time);

private:
  // acc = g (pitch_angle + tilt_angle)
  enum AccelerationMode {
    STOP_BEFORE_PITCH_LIMIT,      // 最大tilt角速度 + 最大pitch rateで加速, 最大pitch角になる前に加速度増加終了
    STOP_AT_PITCH_LIMIT,          // 最大tilt角速度 + 最大pitch rateで加速, 最大pitch角に到達し, その後加速度増加終了
    CONTINUE_BEFORE_PITCH_LIMIT,  // 最大tilt角速度 + 最大pitch rateで加速, その後加速度は増加し続ける. 最大pitch角には到達しない
    CONTINUE_AFTER_PITCH_LIMIT,   // 最大tilt角速度 + 最大pitch rateで加速, 最大pitch角に到達し, その後加速度は増加しつづける
  };
  struct StopBeforePitchLimitConfig {
    double duration; // 軌道終了までにかかる時間
    double tc; // 最大加速度に到達する時刻
  };
  struct StopAtPitchLimitConfig {
    double duration; // 軌道終了までにかかる時間
    double tc1; // 最大pitch角に到達する時刻
    double tc2; // 最大加速度に到達する時刻
  };
  struct ContinueBeforePitchLimitConfig {
    double tilt_finish_time;
    double duration; // 軌道終了までにかかる時間 (≠ スラスト角最大になる時間)
  };
  struct ContinueAfterPitchLimitConfig {
    double tilt_finish_time;
    double duration; // 軌道終了までにかかる時間 (≠ スラスト角最大になる時間)
    double tc; // 最大pitch角に到達する時刻
  };
  // mutables
  builtin_interfaces::msg::Time start_time_;
  double start_pitch_angle_;
  double start_tilt_angle_;
  double start_vx_;
  AccelerationMode mode_;
  StopBeforePitchLimitConfig stop_before_pitch_limit_config_;
  StopAtPitchLimitConfig stop_at_pitch_limit_config_;
  ContinueBeforePitchLimitConfig continue_before_pitch_limit_config_;
  ContinueAfterPitchLimitConfig continue_after_pitch_limit_config_;

  // Configs
  double max_tilt_velocity_;
  double max_pitch_;
  double max_pitch_rate_;
  double cruise_speed_;

  double calcTargetVxStopBeforePitchLimit(const double& elapsed_time) const;
  double calcTargetVxStopAtPitchLimit(const double& elapsed_time) const;
  double calcTargetVxContinueBeforePitchLimit(const double& elapsed_time) const;
  double calcTargetVxContinueAfterPitchLimit(const double& elapsed_time) const;
  double calcTargetAccxStopBeforePitchLimit(const double& elapsed_time) const;
  double calcTargetAccxStopAtPitchLimit(const double& elapsed_time) const;
  double calcTargetAccxContinueBeforePitchLimit(const double& elapsed_time) const;
  double calcTargetAccxContinueAfterPitchLimit(const double& elapsed_time) const;
};
} // namespace y_axis_tilt_vtol
} // namespace tobas