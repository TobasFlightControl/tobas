#include "tobas_y_axis_tilt_vtol_controller/backward_transition_trajectory.hpp"

#include <tobas_ros2_tools/time.hpp>

#include "tobas_y_axis_tilt_vtol_controller/constants.hpp"

namespace tobas::y_axis_tilt_vtol
{
BackwardTransitionTrajectory::BackwardTransitionTrajectory()
{
}

void BackwardTransitionTrajectory::setMaxTiltVelocity(const double& _max_tilt_velocity)
{
  max_tilt_velocity_ = _max_tilt_velocity;
}

void BackwardTransitionTrajectory::setStartConditions(
  const builtin_interfaces::msg::Time& _start_time,
  const double& _start_tilt_angle)
{
  start_time_ = _start_time;
  start_tilt_angle_ = _start_tilt_angle;
  duration_ = (kUpwardAngle - start_tilt_angle_) / -max_tilt_velocity_;
  assert(duration_ > 0);
}

double BackwardTransitionTrajectory::calcTargetTiltAngle(const builtin_interfaces::msg::Time& _cur_time)
{
  const auto elapsed_time = (_cur_time - start_time_).seconds();
  if (elapsed_time < duration_) {
    return start_tilt_angle_ - max_tilt_velocity_ * elapsed_time;
  }
  else {
    return kUpwardAngle;
  }
}
}  // namespace tobas::y_axis_tilt_vtol
