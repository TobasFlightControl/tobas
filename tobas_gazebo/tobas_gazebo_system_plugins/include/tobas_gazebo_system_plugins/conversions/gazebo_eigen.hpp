#pragma once

#include <eigen3/Eigen/Core>
#include <gz/math/Vector3.hh>

namespace gazebo
{
template <typename T>
void vector3GazeboToEigen(const gz::math::Vector3<T>& g, Eigen::Matrix<T, 3, 1>& e)
{
  e.x() = g.X();
  e.y() = g.Y();
  e.z() = g.Z();
}

template <typename T>
void vector3EigenToGazebo(const Eigen::Matrix<T, 3, 1>& e, gz::math::Vector3<T>& g)
{
  g.X() = e.x();
  g.Y() = e.y();
  g.Z() = e.z();
}

template <typename T>
void matrix3EigenToGazebo(const Eigen::Matrix<T, 3, 3>& e, gz::math::Matrix3<T>& g)
{
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      g(r, c) = e(r, c);
    }
  }
}

template <typename T>
void matrix3EigenToGazebo(const gz::math::Matrix3<T>& g, Eigen::Matrix<T, 3, 3>& e)
{
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      e(r, c) = g(r, c);
    }
  }
}
}  // namespace gazebo
