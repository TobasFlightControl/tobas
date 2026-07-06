// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/angle_axis_pi.hpp"

#include <iostream>

#include "./util.hpp"

using namespace std;

namespace tobas
{
AngleAxisPI::AngleAxisPI()
{
}

kdl::Vector AngleAxisPI::update(const kdl::Rotation& cur_rot, const kdl::Rotation& tar_rot, const double& dt)
{
  // Compute error in angle-axis form wrt. the local frame
  const auto ep = (cur_rot.inverse() * tar_rot).getRot();

  // Integrate error
  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.0) {
      ei_(i) += ep(i) * dt;
    }
  }

  // Compute target gyro
  return kp_.hadamard(ep) + ki_.hadamard(ei_);
}

bool AngleAxisPI::setProportionalGain(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.0) {
    cerr << "Proportional gain must be non-negative." << endl;
    return false;
  }

  kp_(idx) = value;

  return true;
}

bool AngleAxisPI::setIntegralGain(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.0) {
    cerr << "Integral gain must be non-negative." << endl;
    return false;
  }

  ki_(idx) = value;
  ei_(idx) = 0.0;

  return true;
}
}  // namespace tobas
