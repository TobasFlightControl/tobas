// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace path
{
namespace detail
{
template <typename T>
inline std::string toString(T _x)
{
  return _x;
}

template <>
inline std::string toString(char _c)
{
  return std::string(1, _c);
}
}  // namespace detail

/** Same as `os.path.join()` from Python. */
template <typename T>
std::string join(const T& _x)
{
  return _x;
}

/** Same as `os.path.join()` from Python. */
template <typename T, typename U>
std::string join(const T& _x, const U& _y)
{
  constexpr char sep[] = "/";

  const auto x = detail::toString(_x);
  const auto y = detail::toString(_y);

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

/** Same as `os.path.join()` from Python. */
template <typename T, typename U, typename... Args>
std::string join(const T& _x, const U& _y, const Args&... _args)
{
  return join(join(_x, _y), _args...);
}
}  // namespace path
}  // namespace tobas
