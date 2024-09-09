#pragma once

#include <chrono>

#include <tobas_msgs_adapter/PosVelAccYaw.hpp>

static constexpr double kWaitForTopicRate = 10.;  // [Hz]
static constexpr double kCommandRate = 100.;      // [Hz]

using CommandType = tobas_msgs::PosVelAccYaw;
