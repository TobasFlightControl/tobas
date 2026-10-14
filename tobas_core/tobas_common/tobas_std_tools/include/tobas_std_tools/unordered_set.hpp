// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <unordered_set>

namespace tobas::st
{
/** Return the minimum element. */
template <typename T>
T min(const std::unordered_set<T>& _set)
{
  assert(!_set.empty());

  T res = *_set.begin();
  for (const T& elem : _set) {
    res = min(res, elem);
  }
  return res;
}

/** Return the maximum element. */
template <typename T>
T max(const std::unordered_set<T>& _set)
{
  assert(!_set.empty());

  T res = *_set.begin();
  for (const T& elem : _set) {
    res = max(res, elem);
  }
  return res;
}
}  // namespace tobas::st
