#pragma once

#include <cmath>

namespace tobas_std
{
/* 2つの数値がほとんど等しいときにtrueを返す．GPT4によるとnumpy.isclose()と同じらしい． */
template <typename T>
bool isClose(T x, T y, T abs_tol = 1e-8, T rel_tol = 1e-5)
{
  const auto diff = std::fabs(x - y);
  return diff < abs_tol || diff < rel_tol * std::max(std::fabs(x), std::fabs(y));
}
}  // namespace tobas_std
