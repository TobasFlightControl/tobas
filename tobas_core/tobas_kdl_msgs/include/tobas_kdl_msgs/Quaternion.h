#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/quaternion.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Quaternion_ = tobas_kdl::Quaternion;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<tobas_kdl::Quaternion>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.w);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
