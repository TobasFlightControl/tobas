// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas
{
namespace math
{
/** Return true when two numbers are nearly equal. This is similar to `numpy.isclose()`. */
template <typename T>
inline bool isClose(T _x, T _y, T _abs_tol = 1e-8, T _rel_tol = 1e-5)
{
  const auto diff = std::abs(_x - _y);
  return diff < _abs_tol || diff < _rel_tol * std::max(std::abs(_x), std::abs(_y));
}

/** Check whether a decimal value is an integer. */
template <typename T>
inline bool isInteger(T _x)
{
  double ip;
  return modf(_x, &ip) == 0.0;
}
}  // namespace math
}  // namespace tobas
