// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>

namespace tobas
{
namespace st
{
/* Get the distance from the beginning of the map to an element. */
template <typename T, typename U>
ptrdiff_t getIndex(const std::map<T, U>& mp, const T& key)
{
  const auto it = mp.find(key);
  if (it == mp.end()) {
    return -1;
  }

  return std::distance(mp.begin(), it);
}
}  // namespace st
}  // namespace tobas
