// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas
{
namespace math
{
/* Square a value. */
template <typename T>
inline constexpr T sqr(const T& x) noexcept
{
  return x * x;
}

/* Cube a value. */
template <typename T>
inline constexpr T cube(const T& x) noexcept
{
  return x * x * x;
}

/* Raise a value to the fourth power. */
template <typename T>
inline constexpr T quar(const T& x) noexcept
{
  const auto x2 = sqr(x);
  return sqr(x2);
}

/* Calculate the fourth root. */
template <typename T>
inline constexpr T quart(const T& x) noexcept
{
  const auto sqrt_x = std::sqrt(x);
  return std::sqrt(sqrt_x);
}

/* Return the sign: +1 for positive, -1 for negative, and 0 for zero. */
template <typename T>
inline constexpr int sign(const T& x) noexcept
{
  return (x > 0) - (x < 0);
}

/* Calculate an integer power. */
template <typename T>
inline constexpr T ipow(T base, size_t exp) noexcept
{
  if (exp == 0) {
    return 1;
  }
  if (exp == 1) {
    return base;
  }

  const auto half = ipow(base, exp / 2);
  const auto sqr = half * half;
  return (exp & 1) ? (sqr * base) : sqr;
}

/* Remap x from the range [a, b] to the range [c, d]. */
template <typename T>
inline T remap(T x, T a, T b, T c, T d) noexcept
{
  return a == b ? (c + d) / 2 : (c * (b - x) + d * (x - a)) / (b - a);
}

/* Round up to the given unit. */
inline double ceil(double x, double unit = 1.0) noexcept
{
  return std::ceil(x / unit) * unit;
}

/* Round down to the given unit. */
inline double floor(double x, double unit = 1.0) noexcept
{
  return std::floor(x / unit) * unit;
}

/* Convert a value to the range [-n, n) without changing its remainder modulo 2n. */
template <typename T>
T wrap(T x, T n) noexcept
{
  const auto n2 = 2 * n;

  // Add a half period for the following steps.
  x += n;

  // Convert x to the range [-2n, 2n).
  if constexpr (std::is_floating_point<T>::value) {
    x = std::fmod(x, n2);
  }
  else {
    x = x % n2;
  }

  // Correct the range if x is negative.
  if (x < 0) {
    x += n2;
  }

  // Convert from [0, 2n) to [-n, n).
  // This also cancels the half period added at the start.
  return x - n;
}
}  // namespace math
}  // namespace tobas
