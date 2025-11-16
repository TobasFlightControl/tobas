#pragma once

#include <cmath>

#define M_PI_3 (M_PI / 3)
#define M_2PI (2 * M_PI)

namespace math
{
static constexpr double kDeg2Rad = M_PI / 180;  // degree -> radian
static constexpr double kRpm2Rps = M_PI / 30;   // rpm -> rad/s
}  // namespace math
