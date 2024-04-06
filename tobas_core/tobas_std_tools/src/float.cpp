#include <cmath>

#include "../include/tobas_std_tools/float.hpp"

namespace tobas_std
{
float decodeBinary32(uint32_t bin)
{
  // exponentは符号付き整数型でなければならない
  // exponentが符号なしだと続く(exponent - 150)がオーバーフローしてしまう
  const int sign = bin >> 31 ? -1 : 1;
  const int exponent = (bin >> 23) & 0xFF;
  const int mantissa = (exponent == 0) ? (bin & 0x7FFFFF) << 1 : (bin & 0x7FFFFF) | 0x800000;
  return sign * mantissa * pow(2, exponent - 150);
}
}  // namespace tobas_std
