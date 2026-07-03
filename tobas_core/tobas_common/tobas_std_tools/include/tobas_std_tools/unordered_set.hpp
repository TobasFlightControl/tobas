// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <unordered_set>
#include <vector>

namespace tobas
{
namespace st
{
/* Return the minimum element. */
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

/* Return the maximum element. */
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
}  // namespace st
}  // namespace tobas
