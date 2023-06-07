#pragma once

#include <cstdint>
#include <Eigen/Core>

namespace state_estimation_cascade
{
static const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

static constexpr uint32_t kPosIdx = 0;
static constexpr uint32_t kAltIdx = kPosIdx + 2;
static constexpr uint32_t kVelIdx = kPosIdx + 3;
static constexpr uint32_t kAccIdx = kVelIdx + 3;
static constexpr uint32_t kGravIdx = kAccIdx + 3;
static constexpr uint32_t kStateSize = kGravIdx + 3;
static constexpr uint32_t kInputSize = 6;
static constexpr uint32_t kOutputSize = 9;

static constexpr double kTimerPeriod = 5.;  // [s]

static constexpr bool kDefaultUseGps = true;
static constexpr int kDefaultImuBufSize = 1;
static constexpr int kDefaultMagBufSize = 1;
static constexpr int kDefaultBarBufSize = 1;
static constexpr int kDefaultGpsBufSize = 1;
static constexpr int kDefaultVelBufSize = 1;
}  // namespace state_estimation_cascade
