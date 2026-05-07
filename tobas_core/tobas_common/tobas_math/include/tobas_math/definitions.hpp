// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

#define M_PI_3 (M_PI / 3)
#define M_2PI (2 * M_PI)

namespace tobas
{
namespace math
{
static constexpr double kDeg2Rad = M_PI / 180;  // degree -> radian
static constexpr double kRpm2Rps = M_PI / 30;   // rpm -> rad/s
}  // namespace math
}  // namespace tobas
