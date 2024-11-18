#pragma once

#include <tobas_msgs_adapter/pos_vel_acc_yaw.hpp>

static constexpr double kCommandRate = 100.;  // [Hz]

using CommandType = tobas_msgs::PosVelAccYaw;
