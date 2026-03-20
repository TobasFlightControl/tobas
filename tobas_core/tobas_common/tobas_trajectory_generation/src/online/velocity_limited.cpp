#include "tobas_trajectory_generation/online/velocity_limited.hpp"

#include <cassert>

#include <tobas_math/core.hpp>

namespace traj
{
VelocityLimitedOnlineTrajectoryGenerator::VelocityLimitedOnlineTrajectoryGenerator()
{
}

void VelocityLimitedOnlineTrajectoryGenerator::setMaxVelocity(double max_vel)
{
  assert(max_vel > 0);
  max_vel_ = max_vel;
}

double VelocityLimitedOnlineTrajectoryGenerator::update(double dt)
{
  assert(std::isfinite(max_vel_));
  assert(dt >= 0);

  const auto max_delta = max_vel_ * dt;

  auto delta = tar_pos_ - traj_pos_;
  if (delta > max_delta) {
    delta = max_delta;
    is_saturated_ = true;
  }
  else if (delta < -max_delta) {
    delta = -max_delta;
    is_saturated_ = true;
  }
  else {
    is_saturated_ = false;
  }

  traj_pos_ += delta;

  return traj_pos_;
}

double VelocityLimitedOnlineTrajectoryGenerator::setTargetPointAndUpdate(double tar_pos, double dt)
{
  setTargetPosition(tar_pos);
  return update(dt);
}

void VelocityLimitedOnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double pos)
{
  traj_pos_ = pos;
}
}  // namespace traj
