#pragma once

#include <Eigen/Core>

#include <multirotor_msgs/LinearVelocity.h>

namespace tf
{
void linVelMsgToEigen(const multirotor_msgs::LinearVelocity& m, Eigen::Vector3d& e);

void linVelEigenToMsg(const Eigen::Vector3d& e, multirotor_msgs::LinearVelocity& m);
}  // namespace tf
