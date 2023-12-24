#pragma once

#include <map>

namespace dh_std
{
/**
 * @brief 辞書がキーを持っているか調べる．
 */
template <typename T, typename U>
inline bool contains(const std::map<T, U>& map, const T& key)
{
  return map.find(key) != map.end();
}
}  // namespace dh_std
