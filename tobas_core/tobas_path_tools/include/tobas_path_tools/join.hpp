#pragma once

#include <string>

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
  std::string x = _x;
  std::string y = _y;

  if (y.starts_with('/'))
    return y;
  else if (x.empty() || x.ends_with('/'))
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
