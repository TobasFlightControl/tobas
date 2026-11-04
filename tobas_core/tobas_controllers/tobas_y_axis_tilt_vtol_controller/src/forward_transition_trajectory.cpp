#include "tobas_y_axis_tilt_vtol_controller/forward_transition_trajectory.hpp"

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
{
ForwardTransitionTrajectory::ForwardTransitionTrajectory()
{
}

void ForwardTransitionTrajectory::setMaxTiltVelocity(const double& max_tilt_velocity)
{
  max_tilt_velocity_ = max_tilt_velocity;
}

void ForwardTransitionTrajectory::setCruiseVelocity(const double& cruise_speed)
{
  cruise_speed_ = cruise_speed;
}

void ForwardTransitionTrajectory::setStartConditions(const builtin_interfaces::msg::Time& start_time, const double& start_tilt_angle, const double& start_vx)
{
  start_time_ = start_time;
  start_tilt_angle_ = start_tilt_angle;
  start_vx_ = start_vx;
  duration_ = (kForwardAngle - start_tilt_angle_) / max_tilt_velocity_;
  assert(duration_ > 0);
  target_accx_ = (cruise_speed_ - start_vx_) / duration_;
}

double ForwardTransitionTrajectory::calcTargetTiltAngle(const builtin_interfaces::msg::Time& cur_time)
{
  const auto elapsed_time = (cur_time - start_time_).seconds();
  if (elapsed_time < duration_) {
    return start_tilt_angle_ + max_tilt_velocity_ * elapsed_time;
  } else {
    return kForwardAngle;
  }
}

double ForwardTransitionTrajectory::calcTargetVx(const builtin_interfaces::msg::Time& cur_time)
{
  const auto elapsed_time = (cur_time - start_time_).seconds();
  if (elapsed_time < duration_) {
    return start_vx_ + target_accx_ * elapsed_time;
  } else {
    return cruise_speed_;
  }
}

double ForwardTransitionTrajectory::calcTargetAccx(const builtin_interfaces::msg::Time& cur_time)
{
  const auto elapsed_time = (cur_time - start_time_).seconds();
  if (elapsed_time < duration_) {
    return target_accx_;
  } else {
    return 0.0;
  }
}
} // namespace y_axis_tilt_vtol
} // namespace tobas
