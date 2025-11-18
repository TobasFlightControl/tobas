#pragma once

#include <cassert>
#include <unordered_set>
#include <vector>

namespace tbs
{
/* 要素の最小値を返す． */
template <typename T>
T min(const std::unordered_set<T>& set)
{
  assert(!set.empty());

  T res = *set.begin();
  for (const T& elem : set) {
    res = min(res, elem);
  }
  return res;
}

/* 要素の最大値を返す． */
template <typename T>
T max(const std::unordered_set<T>& set)
{
  assert(!set.empty());

  T res = *set.begin();
  for (const T& elem : set) {
    res = max(res, elem);
  }
  return res;
}
}  // namespace tbs
