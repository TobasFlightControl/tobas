#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/twist.hpp>

#include "./Vector.h"

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Twist_ = tobas_kdl::Twist;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<tobas_kdl::Twist>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.vel);
    stream.next(m.rot);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
