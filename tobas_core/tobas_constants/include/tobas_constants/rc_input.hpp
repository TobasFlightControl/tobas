#pragma once

#include <cstddef>

namespace tobas
{
static constexpr size_t kRcChannelRoll = 0;     // CH1
static constexpr size_t kRcChannelPitch = 1;    // CH2
static constexpr size_t kRcChannelThrot = 2;    // CH3
static constexpr size_t kRcChannelYaw = 3;      // CH4
static constexpr size_t kRcChannelMode = 4;     // CH5
static constexpr size_t kRcChannelSubMode = 5;  // CH6
static constexpr size_t kRcChannelEnable = 6;   // CH7
static constexpr size_t kRcChannelKill = 7;     // CH8
static constexpr size_t kRcChannelGpsw = 8;     // CH9-16

static constexpr size_t kMinSbusChannels = 8;
static constexpr size_t kMaxSbusChannels = 16;
static constexpr size_t kMaxNumOfGpsw = kMaxSbusChannels - kMinSbusChannels;

static constexpr double kRcInputMin = -1.;
static constexpr double kRCInputMid = 0.;
static constexpr double kRcInputMax = 1.;
}  // namespace tobas
