#pragma once

#include <chrono>

#include <tobas_msgs_adapter/PosVelAccYaw.hpp>

namespace tobas_mr_actions
{
static constexpr double kCommandRate = 100.;  // [Hz]
static constexpr auto kWaitForTopic = std::chrono::seconds(1);

using CommandType = tobas_msgs::PosVelAccYaw;
}  // namespace tobas_mr_actions
