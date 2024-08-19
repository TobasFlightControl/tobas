#pragma once

#include <tobas_msgs/PosVelAccYaw.hpp>

namespace tobas_mr_actions
{
static constexpr double kCommandRate = 100.;  // [Hz]

using CommandType = tobas_msgs::PosVelAccYaw;
}  // namespace tobas_mr_actions
