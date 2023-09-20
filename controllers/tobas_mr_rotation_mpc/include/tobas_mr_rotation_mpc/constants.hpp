#pragma once

#include <cinttypes>
#include <cmath>

namespace tobas_mr_rotation_mpc
{
static constexpr uint32_t kRotIdx = 0;
static constexpr uint32_t kGyroIdx = kRotIdx + 3;
static constexpr uint32_t kHForceIdx = kGyroIdx + 3;
static constexpr uint32_t kStateSize = kHForceIdx + 3;
static constexpr uint32_t kCtrlSize = kGyroIdx + 3;

static constexpr uint32_t kRollIdx = kRotIdx;
static constexpr uint32_t kPitchIdx = kRollIdx + 1;
static constexpr uint32_t kYawIdx = kPitchIdx + 1;

static constexpr double kMaxAttitude = M_PI / 3;  // [rad]
static constexpr double kMaxHeadingError = M_PI;  // [rad]

static constexpr double kHMomentScale = 1.;  // テキトー．X500V2で1e-2くらい．
}  // namespace tobas_mr_rotation_mpc
