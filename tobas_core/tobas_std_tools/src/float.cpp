#include <cmath>

#include "../include/tobas_std_tools/float.hpp"

using namespace std;

namespace tobas_std
{
bool isClose(const double& x, const double& y, const double& abs_tol, const double& rel_tol)
{
  const auto diff = abs(x - y);
  return diff < abs_tol || diff < rel_tol * max(abs(x), abs(y));
}

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
