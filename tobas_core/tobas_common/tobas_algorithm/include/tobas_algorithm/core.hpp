// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>

namespace tobas
{
namespace algo
{
/* Calculate the maximum of variadic arguments. Base case. */
template <typename T>
inline T max(T t)
{
  return t;
}

/* Calculate the maximum of variadic arguments. */
template <typename T, typename... Args>
inline T max(T t, Args... args)
{
  return std::max(t, max(args...));
}

/* Calculate the minimum of variadic arguments. Base case. */
template <typename T>
inline T min(T t)
{
  return t;
}

/* Calculate the minimum of variadic arguments. */
template <typename T, typename... Args>
inline T min(T t, Args... args)
{
  return std::min(t, min(args...));
}

/* Convert an angle to the range [-pi, pi). */
double wrapPi(double angle);

/* Limit the L2 norm of a 2D vector without changing its direction. */
void clamp2d(double& x, double& y, const double& max_length);
}  // namespace algo
}  // namespace tobas
