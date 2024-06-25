#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/euler.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Euler_ = kdl::Euler;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<kdl::Euler>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
