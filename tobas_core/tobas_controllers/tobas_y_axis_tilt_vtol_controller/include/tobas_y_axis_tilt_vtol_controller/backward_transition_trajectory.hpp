#pragma once

#include <builtin_interfaces/msg/time.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
{
class BackwardTransitionTrajectory
{
  static constexpr double kUpwardAngle = 0.0;  // [rad]
public:
  explicit BackwardTransitionTrajectory();
  void setMaxTiltVelocity(const double& max_tilt_velocity);
  void setStartConditions(const builtin_interfaces::msg::Time& start_time, const double& start_tilt_angle);
  double calcTargetTiltAngle(const builtin_interfaces::msg::Time& cur_time);

private:
  // mutables
  builtin_interfaces::msg::Time start_time_;
  double start_tilt_angle_;
  double duration_;

  // Configs
  double max_tilt_velocity_;
};
}  // namespace y_axis_tilt_vtol
}  // namespace tobas
