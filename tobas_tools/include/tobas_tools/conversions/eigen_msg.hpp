#pragma once

#include <Eigen/Core>

#include <tobas_msgs/LinearVelocity.h>
#include <tobas_msgs/AngularVelocity.h>
#include <tobas_msgs/Euler.h>

namespace tf
{
void linVelMsgToEigen(const tobas_msgs::LinearVelocity& m, Eigen::Vector3d& e);

void linVelEigenToMsg(const Eigen::Vector3d& e, tobas_msgs::LinearVelocity& m);

void angVelMsgToEigen(const tobas_msgs::AngularVelocity& m, Eigen::Vector3d& e);

void angVelEigenToMsg(const Eigen::Vector3d& e, tobas_msgs::AngularVelocity& m);

void eulerMsgToEigen(const tobas_msgs::Euler& m, Eigen::Vector3d& e);

void eulerEigenToMsg(const Eigen::Vector3d& e, tobas_msgs::Euler& m);
}  // namespace tf
