#pragma once

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
}  // namespace math
