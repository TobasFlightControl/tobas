#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/vector.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Vector_ = tobas_kdl::Vector;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<tobas_kdl::Vector>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data.x());
    stream.next(m.data.y());
    stream.next(m.data.z());
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
