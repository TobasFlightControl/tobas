// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tuple>

namespace tobas
{
namespace st
{
/**
 * Calculate a quaternion (X,Y,Z,W) from ZYX Euler angles (Roll,Pitch,Yaw).
 *
 * @see https://qiita.com/aa_debdeb/items/abe90a9bd0b4809813da
 */
std::tuple<double, double, double, double>
quaternionFromEuler(const double& roll, const double& pitch, const double& yaw);

/**
 * Calculate ZYX Euler angles (Roll,Pitch,Yaw) from a quaternion (X,Y,Z,W).
 *
 * @see https://qiita.com/aa_debdeb/items/abe90a9bd0b4809813da
 */
std::tuple<double, double, double>
eulerFromQuaternion(const double& x, const double& y, const double& z, const double& w);
}  // namespace st
}  // namespace tobas
