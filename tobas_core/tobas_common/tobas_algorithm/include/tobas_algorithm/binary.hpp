#pragma once

#include <cstdint>
#include <stdfloat>

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

/**
 * @brief Decode IEEE 754 half precision floating point number.
 *
 * @note Use std::bit_cast<std::float16_t>(std::float16_t) instead.
 */
std::float16_t decodeR16(uint16_t bin);

/**
 * @brief Decode IEEE 754 single precision floating point number.
 *
 * @note Use std::bit_cast<std::float32_t>(uint32_t) instead.
 */
std::float32_t decodeR32(uint32_t bin);
}  // namespace algo
}  // namespace tobas
