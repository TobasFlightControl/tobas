#pragma once

#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>

#include <tobas_msgs/Odometry.h>

namespace tobas
{
void transformTobasToMsg(const tobas_msgs::Pose& t, geometry_msgs::Transform& m);
void poseTobasToMsg(const tobas_msgs::Pose& t, geometry_msgs::Pose& m);
void odometryTobasToMsg(const tobas_msgs::Odometry& t, nav_msgs::Odometry& m);
}  // namespace tobas
