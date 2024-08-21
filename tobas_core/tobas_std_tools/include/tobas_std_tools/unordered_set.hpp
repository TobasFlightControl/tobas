#pragma once

#include <unordered_set>
#include <vector>
#include <cassert>

namespace tobas_std
{
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
