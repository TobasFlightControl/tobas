#pragma once

#include <cmath>
#include <initializer_list>

#include "./core.hpp"

namespace math
{
/**
 * @brief L2ノルム．
 * cf. 可変長引数テンプレート: https://marycore.jp/prog/cpp/variadic-function/
 */
template <typename... T>
double norm(T... args)
{
  // 引数の個数は少ない想定なので，Kahanの加算アルゴリズムは使わない．
  double squared_sum = 0.;
  for (const auto& x : std::initializer_list<double>{ args... }) {
    squared_sum += math::sqr(x);
  }
  return sqrt(squared_sum);
}
}  // namespace math
