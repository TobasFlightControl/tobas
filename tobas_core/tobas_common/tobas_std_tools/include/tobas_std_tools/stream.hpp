// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <sstream>
#include <string>

namespace tobas
{
namespace st
{
template <typename T>
inline void addToStream(std::stringstream& ss, const T& t)
{
  ss << t;
}

template <typename T, typename... Args>
inline void addToStream(std::stringstream& ss, const T& t, const Args&... args)
{
  ss << t;
  addToStream(ss, args...);
}

template <typename... Args>
inline std::string buildString(const Args&... args)
{
  std::stringstream ss;
  addToStream(ss, args...);
  return ss.str();
}
}  // namespace st
}  // namespace tobas
