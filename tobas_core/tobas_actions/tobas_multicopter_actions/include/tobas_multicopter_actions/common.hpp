#pragma once

#include <cmath>

static constexpr double kCommandRate = 100.;               // [Hz]
static constexpr double kAttitudeRecoveryRate = M_PI / 6;  // [rad/s]

/**
 * @brief 一定速度でゼロに向かう関数．
 *
 * x = max(x0 - vt, 0) (x0 >= 0)
 * x = min(x0 + vt, 0) (x0 <= 0)
 */
double approachZeroLinear(double x0, double v, double t);
