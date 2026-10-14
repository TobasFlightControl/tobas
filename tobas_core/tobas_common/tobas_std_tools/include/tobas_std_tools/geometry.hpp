// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tuple>

namespace tobas::st
{
/**
 * Calculate a quaternion (X,Y,Z,W) from ZYX Euler angles (Roll,Pitch,Yaw).
 *
 * @see https://qiita.com/aa_debdeb/items/abe90a9bd0b4809813da
 */
std::tuple<double, double, double, double> quaternionFromEuler(double _roll, double _pitch, double _yaw);

/**
 * Calculate ZYX Euler angles (Roll,Pitch,Yaw) from a quaternion (X,Y,Z,W).
 *
 * @see https://qiita.com/aa_debdeb/items/abe90a9bd0b4809813da
 */
std::tuple<double, double, double> eulerFromQuaternion(double _x, double _y, double _z, double _w);
}  // namespace tobas::st
