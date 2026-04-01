// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <sys/types.h>

#include <map>

namespace tobas
{
namespace st
{
/**
 * @brief マップの要素の先頭からの距離を取得する．
 */
template <typename T, typename U>
ssize_t getIndex(const std::map<T, U>& mp, const T& key)
{
  const auto it = mp.find(key);
  if (it == mp.end()) {
    return -1;
  }

  return std::distance(mp.begin(), it);
}
}  // namespace st
}  // namespace tobas
