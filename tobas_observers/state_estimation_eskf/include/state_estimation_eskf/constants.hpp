#pragma once

#include <cstdint>
#include <string>

namespace state_estimation_eskf
{
// ノミナル状態の添字
static constexpr size_t kPosIdx = 0;
static constexpr size_t kAltIdx = kPosIdx + 2;
static constexpr size_t kVelIdx = kPosIdx + 3;
static constexpr size_t kQuatIdx = kVelIdx + 3;
static constexpr size_t kAccBiasIdx = kQuatIdx + 4;
static constexpr size_t kGyroBiasIdx = kAccBiasIdx + 3;
static constexpr size_t kGravIdx = kGyroBiasIdx + 3;
static constexpr size_t kStateSize = kGravIdx + 1;

// 誤差状態の添字
static constexpr size_t kDeltaPosIdx = 0;
static constexpr size_t kDeltaAltIdx = kDeltaPosIdx + 2;
static constexpr size_t kDeltaVelIdx = kDeltaPosIdx + 3;
static constexpr size_t kDeltaThetaIdx = kDeltaVelIdx + 3;
static constexpr size_t kDeltaAccBiasIdx = kDeltaThetaIdx + 3;
static constexpr size_t kDeltaGyroBiasIdx = kDeltaAccBiasIdx + 3;
static constexpr size_t kDeltaGravIdx = kDeltaGyroBiasIdx + 3;
static constexpr size_t kDeltaStateSize = kDeltaGravIdx + 1;

// rosparamのデフォルト
static constexpr bool kDefaultUseBarometer = false;
static constexpr bool kDefaultUseGps = true;
static constexpr bool kDefaultDoAccBiasEstimation = false;
static constexpr bool kDefaultDoGyroBiasEstimation = true;
static constexpr bool kDefaultDoGravEstimation = true;

// 標準偏差の初期値
// 共分散行列は成長は遅いが収束は割と速いから，大きすぎるくらいで適当に決めてよい
static constexpr double kInitPosStddev = 3.;        // [m]
static constexpr double kInitVelStddev = 1.;        // [m/s]
static constexpr double kInitRotStddev = M_PI_4;    // [rad]
static constexpr double kInitAccBiasStddev = 1.;    // [m/s^2]
static constexpr double kInitGyroBiasStddev = 0.1;  // [rad/s]
static constexpr double kInitGravStddev = 0.1;      // [m/s^2]

// その他定数
static constexpr double kWarnPeriod = 1.;               // [s]
static constexpr double kPrintStddevPeriod = 1.;        // [s]
static constexpr double kImuTimeGapThreshold = 0.05;    // [s]
static constexpr double kAnormalyScoreThreshold = 10.;  // [-]
}  // namespace state_estimation_eskf
