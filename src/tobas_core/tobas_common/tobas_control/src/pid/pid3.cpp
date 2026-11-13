// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/pid/pid3.hpp"

namespace tobas::ctrl
{
PID3::PID3()
{
}

Eigen::Vector3d PID3::update(
  const Eigen::Vector3d& _cur_pos,
  const Eigen::Vector3d& _cur_vel,
  const Eigen::Vector3d& _tar_pos,
  const Eigen::Vector3d& _tar_vel,
  double _dt)
{
  assert(_dt >= 0);

  const Eigen::Vector3d ep = _tar_pos - _cur_pos;
  const Eigen::Vector3d ed = _tar_vel - _cur_vel;

  // Accumulate the integral error.
  ei_ += ep * _dt;

  // Anti-windup.
  ei_ = ei_.cwiseMax(-i_max).cwiseMin(i_max);

  // PID
  return kp.cwiseProduct(ep) + ki.cwiseProduct(ei_) + kd.cwiseProduct(ed);
}
}  // namespace tobas::ctrl
