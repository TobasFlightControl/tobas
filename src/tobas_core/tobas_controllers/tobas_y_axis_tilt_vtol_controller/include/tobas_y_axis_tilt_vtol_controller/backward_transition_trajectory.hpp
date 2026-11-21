#pragma once

#include <builtin_interfaces/msg/time.hpp>

namespace tobas::y_axis_tilt_vtol
{
class BackwardTransitionTrajectory
{
public:
  explicit BackwardTransitionTrajectory();
  void setMaxTiltVelocity(const double& _max_tilt_velocity);
  void setStartConditions(const builtin_interfaces::msg::Time& _start_time, const double& _start_tilt_angle);
  double calcTargetTiltAngle(const builtin_interfaces::msg::Time& _cur_time);

private:
  // mutables
  builtin_interfaces::msg::Time start_time_;
  double start_tilt_angle_;
  double duration_;

  // Configs
  double max_tilt_velocity_;
};
}  // namespace tobas::y_axis_tilt_vtol
