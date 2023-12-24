#pragma once

#include <tobas_kdl/frame.hpp>

#include <tobas_msgs/Pose.h>

namespace tobas
{
void poseKDLToTobas(const KDL::Frame& k, tobas_msgs::Pose& t);
void poseTobasToKDL(const tobas_msgs::Pose& t, KDL::Frame& k);
}  // namespace tobas
