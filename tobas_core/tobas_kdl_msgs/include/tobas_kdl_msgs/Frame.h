#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/frame.hpp>

#include "./Vector.h"
#include "./Rotation.h"

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Frame_ = kdl::Frame;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<kdl::Frame>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.p);
    stream.next(m.M);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
