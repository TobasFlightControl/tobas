#pragma once

#include <sys/types.h>

#include <map>

namespace tobas_std
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
}  // namespace tobas_std
