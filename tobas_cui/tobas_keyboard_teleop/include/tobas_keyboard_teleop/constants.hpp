#pragma once

#include <chrono>

namespace tobas_keyboard_teleop
{
static constexpr auto kCommandPeriod = std::chrono::milliseconds(10);
static constexpr auto kCheckTopicsTimerPeriod = std::chrono::seconds(5);
static constexpr auto kInstructionTimerPeriod = std::chrono::seconds(10);
static constexpr double kInfoPeriod = 1.;                   // [s]
static constexpr double kWaitForExternalActionServer = 3.;  // [s]
}  // namespace tobas_keyboard_teleop
