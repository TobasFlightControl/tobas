#pragma once

#include <string>

#include "./util.hpp"

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
  const std::string x = _x;
  const std::string y = _y;

  if (starts_with(y, '/'))
    return y;
  else if (x.empty() || ends_with(x, '/'))
    return x + y;
  else
    return x + '/' + y;
}

/* Same as os.path.join() from Python. */
template <typename T, typename U, typename... Args>
std::string join(const T& x, const U& y, const Args&... args)
{
  return join(join(x, y), args...);
}
}  // namespace path
