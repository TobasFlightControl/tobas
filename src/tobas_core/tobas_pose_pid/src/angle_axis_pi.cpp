// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/angle_axis_pi.hpp"

#include <cassert>

namespace tobas
{
AngleAxisPI::AngleAxisPI()
{
}

kdl::Vector AngleAxisPI::update(const kdl::Rotation& _cur_rot, const kdl::Rotation& _tar_rot, double _dt)
{
  // Compute error in angle-axis form wrt. the local frame.
  const auto ep = (_cur_rot.inverse() * _tar_rot).getRot();

  // Integrate error.
  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.0) {
      ei_(i) += ep(i) * _dt;
    }
  }

  // Compute target gyro.
  return kp_.hadamard(ep) + ki_.hadamard(ei_);
}

void AngleAxisPI::setProportionalGain(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  kp_(_idx) = _value;
}

void AngleAxisPI::setIntegralGain(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  ki_(_idx) = _value;
  ei_(_idx) = 0.0;
}
}  // namespace tobas
