#include "tobas_y_axis_tilt_vtol_controller/backward_transition_trajectory.hpp"

namespace tobas
{
namespace y_axis_tilt_vtol
{
BackwardTransitionTrajectory::BackwardTransitionTrajectory()
{
}

void BackwardTransitionTrajectory::setMaxTiltVelocity(const double& max_tilt_velocity)
{
  max_tilt_velocity_ = max_tilt_velocity;
}

void BackwardTransitionTrajectory::setStartConditions(const builtin_interfaces::msg::Time& start_time, const double& start_tilt_angle)
{
  start_time_ = start_time;
  start_tilt_angle_ = start_tilt_angle;
  duration_ = (kUpwardAngle - start_tilt_angle_) / -max_tilt_velocity_;
  assert(duration_ > 0);
}

double BackwardTransitionTrajectory::calcTargetTiltAngle(const builtin_interfaces::msg::Time& cur_time)
{
  const auto elapsed_time = (cur_time - start_time_).seconds();
  if (elapsed_time < duration_) {
    return start_tilt_angle_ - max_tilt_velocity_ * elapsed_time;
  } else {
    return kUpwardAngle;
  }
}

} // namespace y_axis_tilt_vtol
} // namespace tobas
