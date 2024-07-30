#pragma once

#include <ros/serialization.h>

#include <eigen3/Eigen/Core>

namespace tobas_eigen_msgs
{
template <class ContainerAllocator>
using Matrix3d_ = Eigen::Matrix3d;
}  // namespace tobas_eigen_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<Eigen::Matrix3d>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    for (size_t row = 0; row < 3; ++row)
      for (size_t col = 0; col < 3; ++col)
        stream.next(m(row, col));
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
