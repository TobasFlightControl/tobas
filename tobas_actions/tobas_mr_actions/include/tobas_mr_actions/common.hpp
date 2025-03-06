#pragma once

#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>

static constexpr double kCommandRate = 100.;  // [Hz]

using CommandType = tobas_command_msgs::PosVelYaw;
