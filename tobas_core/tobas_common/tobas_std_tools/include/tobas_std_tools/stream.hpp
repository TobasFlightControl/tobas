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
inline void addToStream(std::stringstream& _ss, const T& _t)
{
  _ss << _t;
}

template <typename T, typename... Args>
inline void addToStream(std::stringstream& _ss, const T& _t, const Args&... _args)
{
  _ss << _t;
  addToStream(_ss, _args...);
}

template <typename... Args>
inline std::string buildString(const Args&... _args)
{
  std::stringstream ss;
  addToStream(ss, _args...);
  return ss.str();
}
}  // namespace st
}  // namespace tobas
