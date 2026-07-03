// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas
{
namespace math
{
/* Return true when two numbers are nearly equal. This is similar to `numpy.isclose()`. */
template <typename T>
inline bool isClose(T x, T y, T abs_tol = 1e-8, T rel_tol = 1e-5)
{
  const auto diff = std::abs(x - y);
  return diff < abs_tol || diff < rel_tol * std::max(std::abs(x), std::abs(y));
}

/* Check whether a decimal value is an integer. */
template <typename T>
inline bool isInteger(T x)
{
  double ip;
  return modf(x, &ip) == 0.;
}
}  // namespace math
}  // namespace tobas
