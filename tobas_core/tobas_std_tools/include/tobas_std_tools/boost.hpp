#pragma once

#include <boost/array.hpp>

namespace tobas_std
{
/* 加重平均をとる． */
template <typename T, typename U, std::size_t N>
T average(const boost::array<T, N>& arr, const boost::array<U, N>& weights)
{
  T num = 0;
  U den = 0;
  for (size_t i = 0; i < arr.size(); ++i)
  {
    num += arr[i] * weights[i];
    den += weights[i];
  }

  T res = num / den;
  return res;
}

/* 平均をとる． */
template <typename T, std::size_t N>
T average(const boost::array<T, N>& arr)
{
  boost::array<double, N> weights;
  weights.fill(1);
  return average(arr, weights);
}

/* 3x3行列の対角要素を埋める． */
template <typename T>
inline void fillMatrix3Diag(boost::array<T, 9>& m, const T& v)
{
  m[0] = m[4] = m[8] = v;
}

/* 3x3行列のトレースを計算する． */
template <typename T>
inline T trace(const boost::array<T, 9>& m)
{
  return m[0] + m[4] + m[8];
}

/* 配列全体の型変換． */
template <typename After, typename Before, size_t N>
boost::array<After, N> cast(const boost::array<Before, N>& arr)
{
  boost::array<After, N> res;
  for (size_t i = 0; i < N; ++i)
    res[i] = static_cast<After>(arr[i]);
  return res;
}

template <typename T, size_t N>
boost::array<T, N> operator*(const boost::array<T, N>& lhs, const T& rhs)
{
  boost::array<T, N> res;
  for (size_t i = 0; i < N; ++i)
    res[i] = lhs[i] * rhs;
  return res;
}

template <typename T, size_t N>
boost::array<T, N> operator*(const T& lhs, const boost::array<T, N>& rhs)
{
  return rhs * lhs;
}

template <typename T, size_t N>
boost::array<T, N> operator/(const boost::array<T, N>& lhs, const T& rhs)
{
  assert(rhs != 0);

  boost::array<T, N> res;
  for (size_t i = 0; i < N; ++i)
    res[i] = lhs[i] / rhs;
  return res;
}

template <typename T, size_t N>
boost::array<T, N> operator/(const T& lhs, const boost::array<T, N>& rhs)
{
  boost::array<T, N> res;
  for (size_t i = 0; i < N; ++i)
  {
    assert(rhs[i] != 0);
    res[i] = lhs / rhs[i];
  }
  return res;
}

template <typename T, size_t N>
boost::array<T, N> operator+(const boost::array<T, N>& lhs, const boost::array<T, N>& rhs)
{
  boost::array<T, N> res;
  for (size_t i = 0; i < N; ++i)
    res[i] = lhs[i] + rhs[i];
  return res;
}
}  // namespace tobas_std
