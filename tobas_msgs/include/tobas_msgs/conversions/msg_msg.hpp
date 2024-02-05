#pragma once

#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>

#include <tobas_msgs/Odometry.h>

namespace tobas
{
void transformTobasToMsg(const tobas_msgs::Pose& t, geometry_msgs::Transform& m);
void transformMsgToTobas(const geometry_msgs::Transform& m, tobas_msgs::Pose& t);

void poseTobasToMsg(const tobas_msgs::Pose& t, geometry_msgs::Pose& m);
void poseMsgToTobas(const geometry_msgs::Pose& m, tobas_msgs::Pose& t);

void odometryTobasToMsg(const tobas_msgs::Odometry& t, nav_msgs::Odometry& m);
void odometryMsgToTobas(const nav_msgs::Odometry& m, tobas_msgs::Odometry& t);
}  // namespace tobas
