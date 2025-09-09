#pragma once

#include <algorithm>

namespace algo
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

/* 角度を [-π, π) の範囲に変換する． */
double wrapPi(double angle);

/* 2次元ベクトルの方向を変えないようにL2ノルムを制限する． */
void clamp2d(double& x, double& y, const double& max_length);
}  // namespace algo
