#pragma once

#include <string>

namespace path
{
template <typename T>
std::string join(const T& x)
{
  return x;
}

template <typename T, typename U>
std::string join(const T& x, const U& y)
{
  std::string res = x;
  const std::string& next = y;
  if (!res.empty() && res.back() != '/' && !next.empty() && next.front() != '/')
    res += '/';
  res += next;
  return res;
}

template <typename T, typename U, typename... Args>
std::string join(const T& x, const U& y, const Args&... args)
{
  return join(join(x, y), args...);
}
}  // namespace path
