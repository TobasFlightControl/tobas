#pragma once

#include <vector>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <unordered_set>
#include <functional>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/kahan.hpp>

namespace std
{
/* std::vectorのコンソール出力 */
template <typename T>
ostream& operator<<(ostream& os, const vector<T>& vec)
{
  os << "[";
  for (const auto& x : vec)
    os << x << " ";
  os << "]";
  return os;
}
}  // namespace std

namespace tobas_std
{
/* Naive Summation． The worst-case round-off error scales with O(nε). */
template <typename T>
T sum(const std::vector<T>& arr)
{
  T sum = 0;
  for (const auto& x : arr)
    sum += x;
  return sum;
}

/* Naive Summation． The worst-case round-off error scales with O(nε). */
template <typename T>
T sum(const std::vector<T>& arr, size_t start, size_t size)
{
  const auto stop = start + size;
  assert(stop <= arr.size());

  T sum = 0;
  for (size_t i = start; i < stop; ++i)
    sum += arr[i];
  return sum;
}

/* Kahan Summation. The worst-case round-off error scales with O(nε^2). */
template <typename T>
T fsum(const std::vector<T>& arr)
{
  algo::Kahan<T> sum;
  for (const auto& x : arr)
    sum.add(x);
  return sum.get();
}

/* Kahan Summation. The worst-case round-off error scales with O(nε^2). */
template <typename T>
T fsum(const std::vector<T>& arr, size_t start, size_t size)
{
  const auto stop = start + size;
  assert(stop <= arr.size());

  algo::Kahan<T> sum;
  for (size_t i = start; i < stop; ++i)
    sum.add(arr[i]);
  return sum.get();
}

/* The average of Kahan Summation. */
template <typename T>
T fmean(const std::vector<T>& arr)
{
  if (arr.size() == 0)
    return 0;

  return fsum(arr) / arr.size();
}

/* The average of Kahan Summation. */
template <typename T>
T fmean(const std::vector<T>& arr, size_t start, size_t size)
{
  if (size == 0)
    return 0;

  return fsum(arr, start, size) / size;
}

/* The variance of data. */
template <typename T>
T variance(const std::vector<T>& arr)
{
  if (arr.size() == 0)
    return 0;

  const auto mean = fmean(arr);
  algo::Kahan<T> sum;
  for (const auto& x : arr)
    sum.add(math::sqr(x - mean));
  return sum.get() / arr.size();
}

/* The variance of data. */
template <typename T>
T variance(const std::vector<T>& arr, size_t start, size_t size)
{
  const auto stop = start + size;
  assert(stop <= arr.size());

  if (arr.size() == 0)
    return 0;

  const auto mean = fmean(arr, start, size);
  algo::Kahan<T> sum;
  for (size_t i = start; i < stop; ++i)
    sum.add(math::sqr(arr[i] - mean));
  return sum.get() / size;
}

/* 要素の加重平均をとる． */
template <typename T, typename U>
T average(const std::vector<T>& vec, const std::vector<U>& weights)
{
  assert(vec.size() == weights.size());

  T num = 0;
  U den = 0;
  for (size_t i = 0; i < vec.size(); ++i)
  {
    num += vec[i] * weights[i];
    den += weights[i];
  }

  T res = num / den;
  return res;
}

/* 要素の平均をとる． */
template <typename T>
inline T average(const std::vector<T>& vec)
{
  return average(vec, std::vector<double>(vec.size(), 1.));
}

/* \a std::vector から要素のインデックスを取得する． */
template <typename T>
ssize_t findIndex(const std::vector<T>& vec, const T& item)
{
  const auto ret = std::find(vec.begin(), vec.end(), item);
  if (ret == vec.end())
    return -1;
  return ret - vec.begin();
}

/* 全要素を単一の値で埋める． */
template <typename T>
inline void fill(std::vector<T>& vec, const T& item)
{
  std::fill(vec.begin(), vec.end(), item);
}

/* 全ての要素がaよりも大きいときにtrueを返す． */
template <typename T>
bool all_gt(const std::vector<T>& vec, const T& a)
{
  for (const auto& x : vec)
  {
    if (x <= a)
      return false;
  }
  return true;
}

/* 全ての要素がaよりも小さいときにtrueを返す． */
template <typename T>
bool all_lt(const std::vector<T>& vec, const T& a)
{
  for (const auto& x : vec)
  {
    if (x >= a)
      return false;
  }
  return true;
}

/* 全ての要素がa以上のときにtrueを返す． */
template <typename T>
bool all_ge(const std::vector<T>& vec, const T& a)
{
  for (const auto& x : vec)
  {
    if (x < a)
      return false;
  }
  return true;
}

/* 全ての要素がa以下のときにtrueを返す． */
template <typename T>
bool all_le(const std::vector<T>& vec, const T& a)
{
  for (const auto& x : vec)
  {
    if (x > a)
      return false;
  }
  return true;
}

/* 最も近い値のインデックスを返す． */
template <typename T>
size_t closestIndex(const std::vector<T>& vec, const T& a)
{
  assert(!vec.empty());

  size_t closest_idx = 0;  // コンパイラの警告を防ぐために適当に初期化
  T closest_dist = std::numeric_limits<T>::max();

  for (size_t i = 0; i < vec.size(); ++i)
  {
    const T dist = std::abs(vec[i] - a);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

/* 重複した要素を除去する． */
template <typename T>
std::vector<T> unique(const std::vector<T>& vec)
{
  std::unordered_set<T> seen;
  std::vector<T> res;

  for (const auto& val : vec)
  {
    if (seen.find(val) == seen.end())
    {
      seen.insert(val);
      res.push_back(val);
    }
  }

  return res;
}

/* 全ての要素が一意のときにTrueを返す． */
template <typename T>
inline bool isUnique(const std::vector<T>& vec)
{
  return unique(vec).size() == vec.size();
}

/* 要素を含む場合にTrueを返す． */
template <typename T>
inline bool contains(const std::vector<T>& vec, const T& val)
{
  return std::find(vec.begin(), vec.end(), val) != vec.end();
}

/* 全ての要素が条件を満た場合にTrueを返す． */
template <typename T, typename Lambda>
inline bool allOf(const std::vector<T>& vec, const Lambda& lambda)
{
  return all_of(vec.begin(), vec.end(), lambda);
}

/* 2つのstd::vectorをマージする． */
template <typename T>
std::vector<T> merge(const std::vector<T>& vec1, const std::vector<T>& vec2)
{
  std::vector<T> res = vec1;
  res.insert(res.end(), vec2.begin(), vec2.end());
  return res;
}
}  // namespace tobas_std
