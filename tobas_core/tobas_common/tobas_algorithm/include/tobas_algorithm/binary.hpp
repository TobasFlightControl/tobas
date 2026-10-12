// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstdint>

namespace tobas
{
namespace algo
{
template <typename T>
inline T extractLowerBits(T value, uint8_t n)
{
  const T mask = (static_cast<T>(1) << n) - 1;
  return value & mask;
}

inline int8_t decodeI8(const uint8_t* ptr)
{
  return ptr[0];
}

inline uint8_t decodeU8(const uint8_t* ptr)
{
  return ptr[0];
}

inline int16_t decodeI16(const uint8_t* ptr)
{
  return (ptr[1] << 8) | ptr[0];
}

inline uint16_t decodeU16(const uint8_t* ptr)
{
  return (ptr[1] << 8) | ptr[0];
}

inline int32_t decodeI32(const uint8_t* ptr)
{
  return (ptr[3] << 24) | (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
}

inline uint32_t decodeU32(const uint8_t* ptr)
{
  return (ptr[3] << 24) | (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
}
}  // namespace algo
}  // namespace tobas
