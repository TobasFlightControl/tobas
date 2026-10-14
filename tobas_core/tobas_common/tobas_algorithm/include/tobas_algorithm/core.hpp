// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>

namespace tobas
{
namespace algo
{
/** Calculate the maximum of variadic arguments. Base case. */
template <typename T>
inline T max(T _t)
{
  return _t;
}

/** Calculate the maximum of variadic arguments. */
template <typename T, typename... Args>
inline T max(T _t, Args... _args)
{
  return std::max(_t, max(_args...));
}

/** Calculate the minimum of variadic arguments. Base case. */
template <typename T>
inline T min(T _t)
{
  return _t;
}

/** Calculate the minimum of variadic arguments. */
template <typename T, typename... Args>
inline T min(T _t, Args... _args)
{
  return std::min(_t, min(_args...));
}

/** Convert an angle to the range [-pi, pi). */
double wrapPi(double _angle);

/** Limit the L2 norm of a 2D vector without changing its direction. */
void clamp2d(double& _x, double& _y, double _max_length);
}  // namespace algo
}  // namespace tobas
