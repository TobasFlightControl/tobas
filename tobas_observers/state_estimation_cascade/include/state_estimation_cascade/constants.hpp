#pragma once

#include <cstdint>

namespace state_estimation_cascade
{
static constexpr size_t kPosIdx = 0;
static constexpr size_t kAltIdx = kPosIdx + 2;
static constexpr size_t kVelIdx = kPosIdx + 3;
static constexpr size_t kAccIdx = kVelIdx + 3;
static constexpr size_t kGravIdx = kAccIdx + 3;
static constexpr size_t kStateSize = kGravIdx + 3;
static constexpr size_t kInputSize = 6;
static constexpr size_t kOutputSize = 9;

static constexpr char kFilteredImuTopic[] = "filtered_imu";

static constexpr double kTimerPeriod = 5.;           // [s]
static constexpr double kImuTimeGapThreshold = 0.1;  // [s]

static constexpr bool kDefaultUseGps = true;
static constexpr double kDefaultGpsHorPosStddevThreshold = 0.3;  // [m]
static constexpr double kDefaultGpsVerPosStddevThreshold = 0.6;  // [m]
}  // namespace state_estimation_cascade
