#pragma once

#include <cmath>

namespace math
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

/* 4乗する． */
template <typename T>
constexpr inline T quat(const T& x)
{
  return sqr(sqr(x));
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

/* 与えられた単位で切り上げ． */
inline double ceil(double x, double unit = 1.)
{
  return std::ceil(x / unit) * unit;
}

/* 与えられた単位で切り捨て． */
inline double floor(double x, double unit = 1.)
{
  return std::floor(x / unit) * unit;
}

/* 与えられた数を，2nで割った余りを変えずに[-n, n)の範囲に変換する． */
template <typename T>
T wrap(T x, T n)
{
  const auto n2 = 2 * n;

  // 後の処理のために半周期分足す
  x += n;

  // x を [-2n, 2n) の範囲に変換
  if constexpr (std::is_floating_point<T>::value)
    x = fmod(x, n2);
  else
    x = x % n2;

  // x が負の場合は範囲を補正
  if (x < 0)
    x += n2;

  // [0, 2n) から [-n, n) へ変換
  // 同時に最初に足した半周期を相殺する
  return x - n;
}
}  // namespace math
