#pragma once

#include <Eigen/Geometry>

namespace orientation_estimation_complement
{
void scaleQuaternion(double gain, Eigen::Quaterniond& q);
}
