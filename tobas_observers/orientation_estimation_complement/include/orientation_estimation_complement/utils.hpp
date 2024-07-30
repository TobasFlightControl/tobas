#pragma once

#include <eigen3/Eigen/Geometry>

namespace orientation_estimation_complement
{
void scaleQuaternion(const double& gain, Eigen::Quaterniond& q);
}
