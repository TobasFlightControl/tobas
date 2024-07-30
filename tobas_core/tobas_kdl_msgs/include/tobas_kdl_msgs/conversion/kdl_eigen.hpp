#pragma once

#include <eigen3/Eigen/Geometry>

#include <tobas_kdl_msgs/Quaternion.h>

namespace kdl
{
void quaternionKDLToEigen(const Quaternion& k, Eigen::Quaterniond& e);
void quaternionEigenToKDL(const Eigen::Quaterniond& e, Quaternion& k);
}  // namespace kdl
