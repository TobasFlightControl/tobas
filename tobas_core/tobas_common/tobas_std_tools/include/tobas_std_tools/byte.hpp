// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <cassert>
#include <cinttypes>
#include <cstring>
#include <vector>

namespace tobas
{
namespace st
{
template <typename T>
inline std::vector<uint8_t> toBytes(const T& src)
{
  std::vector<uint8_t> res;
  res.resize(sizeof(T));
  std::memcpy(res.data(), &src, sizeof(T));
  return res;
}

template <typename T>
inline void fromBytes(const std::vector<uint8_t>& src, T& dst)
{
  assert(src.size() >= sizeof(T));
  std::memcpy(&dst, src.data(), sizeof(T));
}

template <typename T>
inline T fromBytes(const std::vector<uint8_t>& src)
{
  T res;
  fromBytes(src, res);
  return res;
}

template <typename T, size_t N>
inline void fromBytes(const std::array<uint8_t, N>& src, T& dst)
{
  static_assert(src.size() >= sizeof(T));
  std::memcpy(&dst, src.data(), sizeof(T));
}

template <typename T, size_t N>
inline T fromBytes(const std::array<uint8_t, N>& src)
{
  T res;
  fromBytes(src, res);
  return res;
}
}  // namespace st
}  // namespace tobas
