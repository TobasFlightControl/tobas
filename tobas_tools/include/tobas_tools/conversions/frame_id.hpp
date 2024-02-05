#pragma once

#include <tobas_kdl/rotation.hpp>

#include <tobas_msgs/PosVelAccYaw.h>

namespace tobas
{
bool changeFrame(const uint8_t& frame_id, const KDL::Rotation& R_gl, tobas_msgs::PosVelAccYaw& msg);
}
