// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas::y_axis_tilt_vtol
{
struct MixerSolution
{
  Eigen::VectorXd thrusts;
  Eigen::VectorXd tilt_angles;
  Eigen::VectorXd cs_deflections;
};
}  // namespace tobas::y_axis_tilt_vtol
