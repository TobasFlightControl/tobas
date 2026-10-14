// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas::eskf
{
/** Estimate heading variance from geomagnetic variance (memo: 2-75). */
double headingVarianceFromMag(const Eigen::Vector3d& _mag, const Eigen::Matrix3d& _cov);
}  // namespace tobas::eskf
