#include "tobas_algorithm/binary.hpp"

#include <cmath>

namespace algo
{
std::float16_t decodeR16(uint16_t bin)
{
  const int16_t sign = bin >> 15 ? -1 : 1;
  const int16_t exponent = (bin >> 10) & 0x1F;
  const int16_t mantissa = (exponent == 0) ? (bin & 0x3FF) << 1 : (bin & 0x3FF) | 0x400;
  return std::ldexp(static_cast<std::float16_t>(sign * mantissa), exponent - 25);
}

std::float32_t decodeR32(uint32_t bin)
{
  const int32_t sign = bin >> 31 ? -1 : 1;
  const int32_t exponent = (bin >> 23) & 0xFF;  // (exponent - 150)のオーバーフローを防ぐために符号付き
  const int32_t mantissa = (exponent == 0) ? (bin & 0x7FFFFF) << 1 : (bin & 0x7FFFFF) | 0x800000;
  return std::ldexp(static_cast<std::float32_t>(sign * mantissa), exponent - 150);
}
}  // namespace algo
