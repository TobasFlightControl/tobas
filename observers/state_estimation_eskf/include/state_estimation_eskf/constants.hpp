#pragma once

#include <cstdint>
#include <string>

#include <dh_std_tools/math.hpp>

namespace state_estimation_eskf
{
// ノミナル状態の添字
static constexpr uint32_t kPosIdx = 0;
static constexpr uint32_t kAltIdx = kPosIdx + 2;
static constexpr uint32_t kVelIdx = kPosIdx + 3;
static constexpr uint32_t kQuatIdx = kVelIdx + 3;
static constexpr uint32_t kAccBiasIdx = kQuatIdx + 4;
static constexpr uint32_t kGyroBiasIdx = kAccBiasIdx + 3;
static constexpr uint32_t kStateSize = kGyroBiasIdx + 3;

// 誤差状態の添字
static constexpr uint32_t kDeltaPosIdx = 0;
static constexpr uint32_t kDeltaAltIdx = kDeltaPosIdx + 2;
static constexpr uint32_t kDeltaVelIdx = kDeltaPosIdx + 3;
static constexpr uint32_t kDeltaThetaIdx = kDeltaVelIdx + 3;
static constexpr uint32_t kDeltaAccBiasIdx = kDeltaThetaIdx + 3;
static constexpr uint32_t kDeltaGyroBiasIdx = kDeltaAccBiasIdx + 3;
static constexpr uint32_t kDeltaStateSize = kDeltaGyroBiasIdx + 3;

// rosparamのデフォルト
static const std::string kDefaultGeomagObserveMethod = "rpy";
static constexpr bool kDefaultUseBarometer = true;
static constexpr bool kDefaultUseGps = true;
static constexpr double kDefaultGpsHorPosStddevThreshold = 0.3;  // [m]
static constexpr double kDefaultGpsVerPosStddevThreshold = 0.6;  // [m]

// 最初の状態チェックの閾値
static constexpr double kVelocityThreshold = 0.2;  // [m/s]

// その他定数
static constexpr double kInfoPeriod = 5.;              // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;  // [s]
static constexpr double kImuTimeGapThreshold = 0.1;    // [s]
static constexpr double kWaitToPublish = 3.;  // 状態安定のためESKFが稼働してから少し待つ [s]
static constexpr double kInitRotStddev = dh_std::deg2rad(10.);  // [rad]
}  // namespace state_estimation_eskf
