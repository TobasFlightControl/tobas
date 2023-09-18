#pragma once

#include <cmath>

namespace tobas_mr_rotation_mpc
{
static constexpr uint32_t kRollIdx = 0;
static constexpr uint32_t kPitchIdx = kRollIdx + 1;
static constexpr uint32_t kYawIdx = kPitchIdx + 1;
static constexpr uint32_t kGyroIdx = kYawIdx + 1;
static constexpr uint32_t kStateSize = kGyroIdx + 3;

static constexpr double kMaxAttitude = M_PI / 3;  // [rad]
static constexpr double kMaxHeadingError = M_PI;  // [rad]
}  // namespace tobas_mr_rotation_mpc
