#pragma once

#include <chrono>

#include <tobas_msgs_adapter/PosVelAccYaw.hpp>

static constexpr double kCommandRate = 100.;  // [Hz]
static constexpr auto kWaitForTopic = std::chrono::seconds(1);

using CommandType = tobas_msgs::PosVelAccYaw;
