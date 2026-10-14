// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_std_tools/geometry.hpp"

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_math/float.hpp>

namespace tobas::st
{
std::tuple<double, double, double, double> quaternionFromEuler(double _roll, double _pitch, double _yaw)
{
  const auto cx = std::cos(0.5 * _roll);
  const auto sx = std::sin(0.5 * _roll);
  const auto cy = std::cos(0.5 * _pitch);
  const auto sy = std::sin(0.5 * _pitch);
  const auto cz = std::cos(0.5 * _yaw);
  const auto sz = std::sin(0.5 * _yaw);

  const auto x = sx * cy * cz - cx * sy * sz;
  const auto y = sx * cy * sz + cx * sy * cz;
  const auto z = -sx * sy * cz + cx * cy * sz;
  const auto w = sx * sy * sz + cx * cy * cz;

  return { x, y, z, w };
}

std::tuple<double, double, double> eulerFromQuaternion(double _x, double _y, double _z, double _w)
{
  assert(math::isClose(math::sqr(_x) + math::sqr(_y) + math::sqr(_z) + math::sqr(_w), 1.0));

  const auto sy = -2 * (_x * _z - _y * _w);

  const auto pitch = std::asin(sy);

  double roll, yaw;
  if (math::isClose(std::abs(sy), 1.0)) {
    roll = 0.0;
    yaw = std::atan2(-2 * (_x * _y - _z * _w), 2 * (math::sqr(_w) + math::sqr(_y)) - 1);
  }
  else {
    roll = std::atan2(2 * (_y * _z + _x * _w), 2 * (math::sqr(_w) + math::sqr(_z)) - 1);
    yaw = std::atan2(2 * (_x * _y + _z * _w), 2 * (math::sqr(_w) + math::sqr(_x)) - 1);
  }

  return { roll, pitch, yaw };
}
}  // namespace tobas::st
