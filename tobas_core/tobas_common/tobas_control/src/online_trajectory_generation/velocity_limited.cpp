#include "tobas_control/online_trajectory_generation/velocity_limited.hpp"

#include <algorithm>
#include <cassert>

#include <tobas_math/core.hpp>

namespace ctrl
{
VelocityLimitedOnlineTrajectoryGenerator::VelocityLimitedOnlineTrajectoryGenerator()
{
}

void VelocityLimitedOnlineTrajectoryGenerator::setMaxVelocity(double max_vel)
{
  assert(max_vel > 0);
  max_vel_ = max_vel;
}

void VelocityLimitedOnlineTrajectoryGenerator::update(double dt, double cur_pos)
{
  assert(std::isfinite(max_vel_));
  assert(max_vel_ > 0);
  assert(dt >= 0);

  const auto max_delta = max_vel_ * dt;
  const auto delta = std::clamp(tar_pos_ - cur_pos, -max_delta, max_delta);
  traj_pos_ = cur_pos + delta;
}

void VelocityLimitedOnlineTrajectoryGenerator::update(double dt)
{
  update(dt, traj_pos_);
}

void VelocityLimitedOnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double pos)
{
  traj_pos_ = pos;
}
}  // namespace ctrl
