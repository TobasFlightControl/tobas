#pragma once

#include <unordered_set>
#include <vector>
#include <cassert>

namespace tobas_std
{
/**
 * @brief 要素を含むか調べる．
 */
template <typename T>
inline bool contains(const std::unordered_set<T>& set, const T& key)
{
  return set.find(key) != set.end();
}

/* std::vectorに変換する． */
template <typename T>
inline std::vector<T> toVector(const std::unordered_set<T>& set)
{
  return std::vector<T>(set.begin(), set.end());
}

/* 要素の最小値を返す． */
template <typename T>
T min(const std::unordered_set<T>& set)
{
  assert(set.size() > 0);

  T res = *set.begin();
  for (const T& elem : set)
    res = min(res, elem);
  return res;
}

/* 要素の最大値を返す． */
template <typename T>
T max(const std::unordered_set<T>& set)
{
  assert(set.size() > 0);

  T res = *set.begin();
  for (const T& elem : set)
    res = max(res, elem);
  return res;
}
}  // namespace tobas_std
