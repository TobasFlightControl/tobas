#pragma once

#include <Eigen/Core>

#include <multirotor_msgs/LinearVelocity.h>
#include <multirotor_msgs/AngularVelocity.h>

namespace tf
{
void linVelMsgToEigen(const multirotor_msgs::LinearVelocity& m, Eigen::Vector3d& e);

void linVelEigenToMsg(const Eigen::Vector3d& e, multirotor_msgs::LinearVelocity& m);

void angVelMsgToEigen(const multirotor_msgs::AngularVelocity& m, Eigen::Vector3d& e);

void angVelEigenToMsg(const Eigen::Vector3d& e, multirotor_msgs::AngularVelocity& m);
}  // namespace tf
