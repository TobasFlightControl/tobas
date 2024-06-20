#pragma once

#include <cmath>
#include <initializer_list>

namespace tobas_std
{
/* 2乗する． */
template <typename T>
constexpr inline T sqr(const T& x)
{
  return x * x;
}

/* 3乗する． */
template <typename T>
constexpr inline T cube(const T& x)
{
  return x * x * x;
}

/* xを[a, b]の範囲から[c, d]の範囲に投影する． */
template <typename T>
inline T remap(T x, T a, T b, T c, T d)
{
  return a == b ? (c + d) / 2 : (c * (b - x) + d * (x - a)) / (b - a);
}

/**
 * @brief L2ノルム．
 * cf. 可変長引数テンプレート: https://marycore.jp/prog/cpp/variadic-function/
 */
template <typename... T>
double norm(T... args)
{
  // TODO: Use Kahan summation
  double squared_sum = 0.;
  for (const auto& x : std::initializer_list<double>{ args... })
    squared_sum += sqr(x);
  return sqrt(squared_sum);
}
}  // namespace tobas_std
