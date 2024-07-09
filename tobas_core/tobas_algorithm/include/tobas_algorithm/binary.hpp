#pragma once

#include <cinttypes>

namespace algo
{
template <typename T>
inline T extractLowerBits(T value, uint8_t n)
{
  const T mask = (static_cast<T>(1) << n) - 1;
  return value & mask;
}

/* Decode IEEE 754 single precision floating point number. */
float decodeBinary32(uint32_t bin);
}  // namespace algo
