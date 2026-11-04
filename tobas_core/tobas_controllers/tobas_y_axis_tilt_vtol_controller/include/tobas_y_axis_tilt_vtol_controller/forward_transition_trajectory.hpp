#pragma once

#include <cmath>

#include <tobas_ros2_tools/time.hpp>

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
  void setMaxTiltVelocity(const double& max_tilt_velocity);
  void setCruiseVelocity(const double& cruise_speed);
  void setStartConditions(const builtin_interfaces::msg::Time& start_time, const double& start_tilt_angle, const double& start_vx);
  double calcTargetTiltAngle(const builtin_interfaces::msg::Time& cur_time);
  double calcTargetVx(const builtin_interfaces::msg::Time& cur_time);
  double calcTargetAccx(const builtin_interfaces::msg::Time& cur_time);

private:
  // mutables
  builtin_interfaces::msg::Time start_time_;
  double start_tilt_angle_;
  double start_vx_;
  double duration_;
  double target_accx_;

  // Configs
  double max_tilt_velocity_;
  double cruise_speed_;
};
} // namespace y_axis_tilt_vtol
} // namespace tobas