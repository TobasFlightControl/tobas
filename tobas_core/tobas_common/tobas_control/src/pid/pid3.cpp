// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/pid/pid3.hpp"

namespace tobas
{
namespace ctrl
{
PID3::PID3()
{
}

Eigen::Vector3d PID3::update(
  const Eigen::Vector3d& cur_pos,
  const Eigen::Vector3d& cur_vel,
  const Eigen::Vector3d& tar_pos,
  const Eigen::Vector3d& tar_vel,
  const double& dt)
{
  assert(dt >= 0);

  const Eigen::Vector3d ep = tar_pos - cur_pos;
  const Eigen::Vector3d ed = tar_vel - cur_vel;

  // Accumulate the integral error.
  ei_ += ep * dt;

  // Anti-windup.
  ei_ = ei_.cwiseMax(-i_max).cwiseMin(i_max);

  // PID
  return kp.cwiseProduct(ep) + ki.cwiseProduct(ei_) + kd.cwiseProduct(ed);
}
}  // namespace ctrl
}  // namespace tobas
