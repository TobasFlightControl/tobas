// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/angle_axis_pi.hpp"

#include <cassert>

namespace tobas
{
AngleAxisPI::AngleAxisPI()
{
}

kdl::Vector AngleAxisPI::update(const kdl::Rotation& cur_rot, const kdl::Rotation& tar_rot, const double& dt)
{
  // Compute error in angle-axis form wrt. the local frame.
  const auto ep = (cur_rot.inverse() * tar_rot).getRot();

  // Integrate error.
  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.0) {
      ei_(i) += ep(i) * dt;
    }
  }

  // Compute target gyro.
  return kp_.hadamard(ep) + ki_.hadamard(ei_);
}

void AngleAxisPI::setProportionalGain(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  kp_(idx) = value;
}

void AngleAxisPI::setIntegralGain(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  ki_(idx) = value;
  ei_(idx) = 0.0;
}
}  // namespace tobas
