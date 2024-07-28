#include <cmath>
#include <cassert>

#include "../include/tobas_algorithm/core.hpp"

namespace algo
{
double wrapPi(double angle)
{
  constexpr double TWO_PI = 2 * M_PI;
  angle = fmod(angle + M_PI, TWO_PI);  // angle を [0, 2π) の範囲に変換
  if (angle < 0)
    angle += TWO_PI;    // angle が負の場合、範囲を補正
  return angle - M_PI;  // [0, 2π) から [-π, π) へ変換
}

void clamp2d(double& x, double& y, const double& max_length)
{
  assert(max_length >= 0);

  const auto length = sqrt(x * x + y * y);
  if (length > max_length)
  {
    const auto scale = max_length / length;
    x *= scale;
    y *= scale;
  }
}
}  // namespace algo
