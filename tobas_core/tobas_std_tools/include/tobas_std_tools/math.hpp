#pragma once

#include <cmath>
#include <limits>
#include <initializer_list>

namespace tobas_std
{
/* ===== 角度の単位の変換．引数が整数の時に0が返るのを避けるためテンプレートにはしない． ===== */

/* degree -> radian */
constexpr inline double deg2rad(const double& deg)
{
  return deg * (M_PI / 180.);
}

/* radian -> degree */
constexpr inline double rad2deg(const double& rad)
{
  return rad * (180. / M_PI);
}

/* rpm -> rad/s */
constexpr inline double rpm2rps(const double& rpm)
{
  return rpm * (M_PI / 30);
}

/* rad/s -> rpm */
constexpr inline double rps2rpm(const double& rad_per_sec)
{
  return rad_per_sec * (30 / M_PI);
}

/* ========== */

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

/* 符号を返す．正なら+1，負なら-1． */
template <typename T>
constexpr inline int sign(const T& x)
{
  return (x > 0) - (x < 0);
}

/* xを[a, b]の範囲から[c, d]の範囲に投影する． */
template <typename T>
inline T remap(T x, T a, T b, T c, T d)
{
  return a == b ? (c + d) / 2 : (c * (b - x) + d * (x - a)) / (b - a);
}

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

/**
 * @brief L2ノルム．
 * cf. 可変長引数テンプレート: https://marycore.jp/prog/cpp/variadic-function/
 */
template <typename... T>
double norm(T... args)
{
  double squared_sum = 0.;
  for (double x : std::initializer_list<double>{ args... })
  {
    squared_sum += sqr(x);
  }
  return sqrt(squared_sum);
}

/* 角度を-πからπの範囲に制限する． */
double wrapPi(double angle);

/* 2つの数値がほとんど等しいときにtrueを返す．GPT4によるとnumpy.isclose()と同じらしい． */
bool isClose(const double& a, const double& b, const double& abs_tol = 1e-8, const double& rel_tol = 1e-5);
}  // namespace tobas_std
