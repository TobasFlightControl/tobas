#pragma once

#include <nav_msgs/Odometry.h>

#include <tobas_msgs/Odometry.h>

namespace tobas
{
void odometryTobasToMsg(const tobas_msgs::Odometry& t, nav_msgs::Odometry& m);
void odometryMsgToTobas(const nav_msgs::Odometry& m, tobas_msgs::Odometry& t);
}  // namespace tobas
