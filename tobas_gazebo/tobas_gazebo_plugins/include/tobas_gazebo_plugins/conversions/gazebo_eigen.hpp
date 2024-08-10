#pragma once

#include <eigen3/Eigen/Core>
#include <gz/math/Vector3.hh>

namespace gazebo
{
template <typename T>
void vectorGazeboToEigen(const gz::math::Vector3<T>& g, Eigen::Matrix<T, 3, 1>& e)
{
  e.x() = g.X();
  e.y() = g.Y();
  e.z() = g.Z();
}

template <typename T>
void vectorEigenToGazebo(const Eigen::Matrix<T, 3, 1>& e, gz::math::Vector3<T>& g)
{
  g.X() = e.x();
  g.Y() = e.y();
  g.Z() = e.z();
}
}  // namespace gazebo
