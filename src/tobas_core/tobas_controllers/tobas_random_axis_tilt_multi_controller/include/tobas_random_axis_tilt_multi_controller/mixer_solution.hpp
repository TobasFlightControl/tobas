// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas::random_axis_tilt_multicopter
{
struct MixerSolution
{
  Eigen::VectorXd thrusts;
  Eigen::VectorXd tilt_angles;
};
}  // namespace tobas::random_axis_tilt_multicopter
