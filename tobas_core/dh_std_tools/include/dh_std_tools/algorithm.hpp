#pragma once

#include <iostream>
#include <string>
#include <algorithm>
#include <cassert>

#include "./console.hpp"

namespace dh_std
{
/* std::clampと機能は同じだが，clampが発生したときに警告を出す． */
template <typename T>
T clamp(const T& x, const T& lb, const T& ub, const std::string& description)
{
  if (lb <= x && x <= ub)
  {
    return x;
  }
  else
  {
    const T res = std::clamp(x, lb, ub);
    DH_WARN("Clamp occurred: " << description << ", " << x << " -> " << res);
    return res;
  }
}

/* 2次元ベクトルの方向を変えないようにL2ノルムを制限する． */
void clamp2d(double& x, double& y, const double& max_length);
}  // namespace dh_std
