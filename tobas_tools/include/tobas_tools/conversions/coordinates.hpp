#pragma once

#include <tobas_msgs/Pose.h>
#include <tobas_msgs/BaseState.h>

namespace tf
{
void poseNedToNwu(const tobas_msgs::Pose& src, tobas_msgs::Pose& des);
void poseNwuToNed(const tobas_msgs::Pose& src, tobas_msgs::Pose& des);
void poseNedToNwu(tobas_msgs::Pose& arg);
void poseNwuToNed(tobas_msgs::Pose& arg);

void baseStateNedToNwu(const tobas_msgs::BaseState& src, tobas_msgs::BaseState& des);
void baseStateNwuToNed(const tobas_msgs::BaseState& src, tobas_msgs::BaseState& des);
void baseStateNedToNwu(tobas_msgs::BaseState& arg);
void baseStateNwuToNed(tobas_msgs::BaseState& arg);
}  // namespace tf
