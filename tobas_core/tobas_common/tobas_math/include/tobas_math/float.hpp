// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas
{
namespace math
{
/* 2つの数値がほとんど等しいときにtrueを返す．GPT4によるとnumpy.isclose()と同じらしい． */
template <typename T>
inline bool isClose(T x, T y, T abs_tol = 1e-8, T rel_tol = 1e-5)
{
  const auto diff = std::abs(x - y);
  return diff < abs_tol || diff < rel_tol * std::max(std::abs(x), std::abs(y));
}

/* 小数が整数部分をもつかどうかを判定する． */
template <typename T>
inline bool isInteger(T x)
{
  double ip;
  return modf(x, &ip) == 0.;
}
}  // namespace math
}  // namespace tobas
