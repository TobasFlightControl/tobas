#pragma once

#include <string>

namespace path
{
/* TODO: C++20が使えるようになったら削除 */
bool starts_with(const std::string& str, const std::string& prefix)
{
  if (prefix.size() > str.size())
    return false;
  return std::equal(prefix.begin(), prefix.end(), str.begin());
}

/* TODO: C++20が使えるようになったら削除 */
bool ends_with(const std::string& str, const std::string& suffix)
{
  if (suffix.size() > str.size())
    return false;
  return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
}
}  // namespace path
