#include "tobas_trajectory_generation/online/velocity_limited.hpp"

#include <algorithm>
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

void VelocityLimitedOnlineTrajectoryGenerator::update(double dt)
{
  assert(std::isfinite(max_vel_));
  assert(dt >= 0);

  const auto max_delta = max_vel_ * dt;
  const auto delta = std::clamp(tar_pos_ - traj_pos_, -max_delta, max_delta);
  traj_pos_ += delta;
}

void VelocityLimitedOnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double pos)
{
  traj_pos_ = pos;
}
}  // namespace traj
