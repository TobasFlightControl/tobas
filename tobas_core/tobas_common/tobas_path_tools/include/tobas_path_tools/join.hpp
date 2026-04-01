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
inline std::string toString(T x)
{
  return x;
}

template <>
inline std::string toString(char c)
{
  return std::string(1, c);
}
}  // namespace detail

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

/* Same as os.path.join() from Python. */
template <typename T, typename U, typename... Args>
std::string join(const T& x, const U& y, const Args&... args)
{
  return join(join(x, y), args...);
}
}  // namespace path
}  // namespace tobas
