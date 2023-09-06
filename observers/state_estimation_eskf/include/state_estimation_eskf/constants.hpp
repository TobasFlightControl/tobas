#pragma once

#include <cstdint>
#include <string>

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
static const std::string kDefaultGeomagObserveMethod = "yaw_only";
static constexpr bool kDefaultUseBarometer = true;
static constexpr bool kDefaultUseGps = true;
static constexpr double kDefaultGpsHorPosStddevThreshold = 0.3;  // [m]
static constexpr double kDefaultGpsVerPosStddevThreshold = 0.6;  // [m]

// 共分散の初期値
// 成長は遅いが収束は割と速いから，大きすぎるくらいで適当に決めてよい
static constexpr double kInitPosStddev = 3.;        // [m]
static constexpr double kInitVelStddev = 1.;        // [m/s]
static constexpr double kInitRotStddev = M_PI_4;    // [rad]
static constexpr double kInitAccBiasStddev = 1.;    // [m/s^2]
static constexpr double kInitGyroBiasStddev = 0.1;  // [rad/s]

// その他定数
static constexpr double kInfoPeriod = 5.;              // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;  // [s]
static constexpr double kImuTimeGapThreshold = 0.05;   // [s]
static constexpr double kWaitToPublish = 3.;           // [s] 共分散の収束を待つ
}  // namespace state_estimation_eskf
