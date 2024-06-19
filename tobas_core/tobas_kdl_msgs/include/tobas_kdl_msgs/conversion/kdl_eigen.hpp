#pragma once

#include <Eigen/Geometry>

#include <tobas_kdl_msgs/Quaternion.h>

namespace tobas_kdl
{
void quaternionKDLToEigen(const Quaternion& k, Eigen::Quaterniond& e);
void quaternionEigenToKDL(const Eigen::Quaterniond& e, Quaternion& k);
}  // namespace tobas_kdl
