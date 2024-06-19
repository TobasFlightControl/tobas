#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/rotation.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Rotation_ = tobas_kdl::Rotation;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<tobas_kdl::Rotation>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    for (size_t row = 0; row < 3; ++row)
      for (size_t col = 0; col < 3; ++col)
        stream.next(m.data(row, col));
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
