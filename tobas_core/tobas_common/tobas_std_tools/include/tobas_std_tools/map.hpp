// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>

namespace tobas::st
{
/** Get the distance from the beginning of the map to an element. */
template <typename T, typename U>
ptrdiff_t getIndex(const std::map<T, U>& _mp, const T& _key)
{
  const auto it = _mp.find(_key);
  if (it == _mp.end()) {
    return -1;
  }

  return std::distance(_mp.begin(), it);
}
}  // namespace tobas::st
