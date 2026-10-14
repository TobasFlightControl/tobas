// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstdint>

namespace tobas
{
namespace algo
{
template <typename T>
inline T extractLowerBits(T _value, uint8_t _n)
{
  const T mask = (static_cast<T>(1) << _n) - 1;
  return _value & mask;
}

inline int8_t decodeI8(const uint8_t* _ptr)
{
  return _ptr[0];
}

inline uint8_t decodeU8(const uint8_t* _ptr)
{
  return _ptr[0];
}

inline int16_t decodeI16(const uint8_t* _ptr)
{
  return (_ptr[1] << 8) | _ptr[0];
}

inline uint16_t decodeU16(const uint8_t* _ptr)
{
  return (_ptr[1] << 8) | _ptr[0];
}

inline int32_t decodeI32(const uint8_t* _ptr)
{
  return (_ptr[3] << 24) | (_ptr[2] << 16) | (_ptr[1] << 8) | _ptr[0];
}

inline uint32_t decodeU32(const uint8_t* _ptr)
{
  return (_ptr[3] << 24) | (_ptr[2] << 16) | (_ptr[1] << 8) | _ptr[0];
}
}  // namespace algo
}  // namespace tobas
