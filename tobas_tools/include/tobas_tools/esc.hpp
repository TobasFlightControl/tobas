#pragma once

#include <tobas_std_tools/enum.hpp>

namespace tobas
{
/* ESC throttle interpretation methods. */
DEFINE_NAMED_ENUM(EscMode);

static constexpr auto BLHELI_OPEN_LOOP = EscMode("BLHELI_OPEN_LOOP", 0);
static constexpr auto BLHELI_CLOSED_LOOP_LOW_RANGE = EscMode("BLHELI_CLOSED_LOOP_LOW_RANGE", 1);
static constexpr auto BLHELI_CLOSED_LOOP_MID_RANGE = EscMode("BLHELI_CLOSED_LOOP_MID_RANGE", 2);
static constexpr auto BLHELI_CLOSED_LOOP_HIGH_RANGE = EscMode("BLHELI_CLOSED_LOOP_HIGH_RANGE", 3);

static constexpr double kBLHeliCLLowMaxERPM = 50000;    // The maximum ERPM in BLHeli closed loop low range mode
static constexpr double kBLHeliCLMidMaxERPM = 100000;   // The maximum ERPM in BLHeli closed loop middle range mode
static constexpr double kBLHeliCLHighMaxERPM = 200000;  // The maximum ERPM in BLHeli closed loop high range mode
}  // namespace tobas
