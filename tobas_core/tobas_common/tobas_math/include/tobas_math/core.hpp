// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas::math
{
/** Square a value. */
template <typename T>
inline constexpr T sqr(const T& _x) noexcept
{
  return _x * _x;
}

/** Cube a value. */
template <typename T>
inline constexpr T cube(const T& _x) noexcept
{
  return _x * _x * _x;
}

/** Raise a value to the fourth power. */
template <typename T>
inline constexpr T quar(const T& _x) noexcept
{
  const auto x2 = sqr(_x);
  return sqr(x2);
}

/** Calculate the fourth root. */
template <typename T>
inline constexpr T quart(const T& _x) noexcept
{
  const auto sqrt_x = std::sqrt(_x);
  return std::sqrt(sqrt_x);
}

/** Return the sign: +1 for positive, -1 for negative, and 0 for zero. */
template <typename T>
inline constexpr int sign(const T& _x) noexcept
{
  return (_x > 0) - (_x < 0);
}

/** Calculate an integer power. */
template <typename T>
inline constexpr T ipow(T _base, size_t _exp) noexcept
{
  if (_exp == 0) {
    return 1;
  }
  if (_exp == 1) {
    return _base;
  }

  const auto half = ipow(_base, _exp / 2);
  const auto sqr = half * half;
  return (_exp & 1) ? (sqr * _base) : sqr;
}

/** Remap x from the range [a, b] to the range [c, d]. */
template <typename T>
inline T remap(T _x, T _a, T _b, T _c, T _d) noexcept
{
  return _a == _b ? (_c + _d) / 2 : (_c * (_b - _x) + _d * (_x - _a)) / (_b - _a);
}

/** Round up to the given unit. */
inline double ceil(double _x, double _unit = 1.0) noexcept
{
  return std::ceil(_x / _unit) * _unit;
}

/** Round down to the given unit. */
inline double floor(double _x, double _unit = 1.0) noexcept
{
  return std::floor(_x / _unit) * _unit;
}

/** Convert a value to the range [-n, n) without changing its remainder modulo 2n. */
template <typename T>
T wrap(T _x, T _n) noexcept
{
  const auto n2 = 2 * _n;

  // Add a half period for the following steps.
  _x += _n;

  // Convert x to the range [-2n, 2n).
  if constexpr (std::is_floating_point<T>::value) {
    _x = std::fmod(_x, n2);
  }
  else {
    _x = _x % n2;
  }

  // Correct the range if x is negative.
  if (_x < 0) {
    _x += n2;
  }

  // Convert from [0, 2n) to [-n, n).
  // This also cancels the half period added at the start.
  return _x - _n;
}
}  // namespace tobas::math
