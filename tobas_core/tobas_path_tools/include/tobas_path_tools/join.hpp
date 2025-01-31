#pragma once

#include <tobas_string_tools/core.hpp>

namespace path
{
/* Same as os.path.join() from Python. */
template <typename T>
std::string join(const T& x)
{
  return x;
}

/* Same as os.path.join() from Python. */
template <typename T, typename U>
std::string join(const T& _x, const U& _y)
{
  constexpr char sep[] = "/";

  const std::string x = _x;
  const std::string y = _y;

  // C++17以下でも動作するようstring::starts_with等は使用しない
  if (str::startsWith(y, sep))
    return y;
  else if (x.empty() || str::endsWith(x, sep))
    return x + y;
  else
    return x + sep + y;
}

/* Same as os.path.join() from Python. */
template <typename T, typename U, typename... Args>
std::string join(const T& x, const U& y, const Args&... args)
{
  return join(join(x, y), args...);
}
}  // namespace path
