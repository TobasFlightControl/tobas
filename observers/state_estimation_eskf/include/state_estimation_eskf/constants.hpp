#pragma once

#include <cstdint>

namespace state_estimation_eskf
{
static constexpr uint32_t kPosIdx = 0;
static constexpr uint32_t kAltIdx = kPosIdx + 2;
static constexpr uint32_t kVelIdx = kPosIdx + 3;
static constexpr uint32_t kQuatIdx = kVelIdx + 3;
static constexpr uint32_t kAccBiasIdx = kQuatIdx + 4;
static constexpr uint32_t kGyroBiasIdx = kAccBiasIdx + 3;
static constexpr uint32_t kStateSize = kGyroBiasIdx + 3;

static constexpr uint32_t kDeltaPosIdx = 0;
static constexpr uint32_t kDeltaAltIdx = kDeltaPosIdx + 2;
static constexpr uint32_t kDeltaVelIdx = kDeltaPosIdx + 3;
static constexpr uint32_t kDeltaThetaIdx = kDeltaVelIdx + 3;
static constexpr uint32_t kDeltaAccBiasIdx = kDeltaThetaIdx + 3;
static constexpr uint32_t kDeltaGyroBiasIdx = kDeltaAccBiasIdx + 3;
static constexpr uint32_t kDeltaStateSize = kDeltaGyroBiasIdx + 3;

static constexpr double kTimerPeriod = 5.;  // [s]
static constexpr double kWaitToPublish = 3.;  // 状態安定のためESKFが稼働してから少し待つ [s]

static constexpr bool kDefaultUseGps = true;
static constexpr int kDefaultImuBufSize = 1;
static constexpr int kDefaultMagBufSize = 1;
static constexpr int kDefaultBarBufSize = 1;
static constexpr int kDefaultGpsBufSize = 1;
static constexpr int kDefaultVelBufSize = 1;
}  // namespace state_estimation_eskf
