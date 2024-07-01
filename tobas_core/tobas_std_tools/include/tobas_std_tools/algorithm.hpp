#pragma once

#include <iostream>
#include <string>
#include <algorithm>
#include <cassert>

#include "./console.hpp"

namespace tobas_std
{
/* 可変長引数の最大値を計算する． (ベースケース) */
template <typename T>
inline T max(T t)
{
  return t;
}

/* 可変長引数の最大値を計算する． */
template <typename T, typename... Args>
inline T max(T t, Args... args)
{
  return std::max(t, max(args...));
}

/* 可変長引数の最小値を計算する． (ベースケース) */
template <typename T>
inline T min(T t)
{
  return t;
}

/* 可変長引数の最小値を計算する． */
template <typename T, typename... Args>
inline T min(T t, Args... args)
{
  return std::min(t, min(args...));
}

/* 2次元ベクトルの方向を変えないようにL2ノルムを制限する． */
void clamp2d(double& x, double& y, const double& max_length);

/* 角度を-πからπの範囲に制限する． */
double wrapPi(double angle);
}  // namespace tobas_std
