// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>
#include <initializer_list>

#include "./core.hpp"

namespace tobas
{
namespace math
{
/**
 * @brief L2 norm.
 * cf. Variadic function templates: https://marycore.jp/prog/cpp/variadic-function/
 */
template <typename... T>
double norm(T... args)
{
  // The number of arguments is expected to be small, so Kahan summation is not used.
  double squared_sum = 0.0;
  for (const auto& x : std::initializer_list<double>{ args... }) {
    squared_sum += math::sqr(x);
  }
  return std::sqrt(squared_sum);
}
}  // namespace math
}  // namespace tobas
