#include "tobas_algorithm/binary.hpp"

#include <cmath>

namespace algo
{
float decodeR32(uint32_t bin)
{
  // exponentは符号付き整数型でなければならない
  // exponentが符号なしだと続く(exponent - 150)がオーバーフローしてしまう
  const int32_t sign = bin >> 31 ? -1 : 1;
  const int32_t exponent = (bin >> 23) & 0xFF;
  const int32_t mantissa = (exponent == 0) ? (bin & 0x7FFFFF) << 1 : (bin & 0x7FFFFF) | 0x800000;
  return sign * mantissa * std::pow(2, exponent - 150);
}
}  // namespace algo
