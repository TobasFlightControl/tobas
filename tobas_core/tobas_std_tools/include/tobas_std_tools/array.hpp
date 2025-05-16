#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <iostream>

#include <tobas_algorithm/kahan.hpp>

namespace std
{
/* std::arrayのコンソール出力 */
template <typename T, size_t N>
std::ostream& operator<<(std::ostream& os, const std::array<T, N>& arr)
{
  if (N == 0) {
    os << "[]";
    return os;
  }

  os << "[";
  for (size_t i = 0; i < N - 1; ++i) {
    os << arr[i] << " ";
  }
  os << arr[N - 1] << "]";

  return os;
}
}  // namespace std

namespace tobas_std
{
/* Naive Summation． The worst-case round-off error scales with O(nε). */
template <typename T, size_t N>
T sum(const std::array<T, N>& arr)
{
  T sum = 0;
  for (const auto& x : arr) {
    sum += x;
  }
  return sum;
}

/* Kahan Summation. The worst-case round-off error scales with O(nε^2). */
template <typename T, size_t N>
T fsum(const std::array<T, N>& arr)
{
  algo::Kahan<T> sum;
  for (const auto& x : arr) {
    sum.add(x);
  }
  return sum.get();
}

/* The average of Kahan Summation. */
template <typename T, size_t N>
T fmean(const std::array<T, N>& arr)
{
  if (N == 0) {
    return 0;
  }
  else {
    return fsum(arr) / N;
  }
}

/* 最も近い値のインデックスを返す． */
template <typename T, size_t N>
size_t closestIndex(const std::array<T, N>& arr, const T& a)
{
  assert(N > 0);

  size_t closest_idx = 0;  // コンパイラの警告を防ぐために適当に初期化
  T closest_dist = std::numeric_limits<T>::max();

  for (size_t i = 0; i < N; ++i) {
    const T dist = std::abs(arr[i] - a);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}
}  // namespace tobas_std
