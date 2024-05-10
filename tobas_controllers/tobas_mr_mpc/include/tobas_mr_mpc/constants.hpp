#pragma once

#include <cstddef>

namespace tobas_mr_mpc
{
// 姿勢制御機のインデックス
static constexpr size_t kRotIdx = 0;
static constexpr size_t kGyroIdx = kRotIdx + 3;
static constexpr size_t kHForceIdx = kGyroIdx + 3;
static constexpr size_t kStateSize = kHForceIdx + 3;
static constexpr size_t kCtrlSize = kGyroIdx + 3;

static constexpr size_t kRollIdx = kRotIdx;
static constexpr size_t kPitchIdx = kRollIdx + 1;
static constexpr size_t kYawIdx = kPitchIdx + 1;

static constexpr double kHMomentScale = 1.;  // テキトー．X500V2で1e-2くらい．
}  // namespace tobas_mr_mpc
