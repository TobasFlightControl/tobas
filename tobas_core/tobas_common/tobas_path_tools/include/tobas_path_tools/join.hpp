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
  constexpr char sep[] = "/";

  const std::string x = _x;
  const std::string y = _y;

  if (y.starts_with(sep)) {
    return y;
  }
  else if (x.empty() || x.ends_with(sep)) {
    return x + y;
  }
  else {
    return x + sep + y;
  }
}

/* Same as os.path.join() from Python. */
template <typename T, typename U, typename... Args>
std::string join(const T& x, const U& y, const Args&... args)
{
  return join(join(x, y), args...);
}
}  // namespace path
