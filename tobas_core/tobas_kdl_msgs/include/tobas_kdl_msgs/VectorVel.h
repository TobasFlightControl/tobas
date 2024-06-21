#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/vectorvel.hpp>

#include "./Vector.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using VectorVel_ = kdl::VectorVel;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<kdl::VectorVel>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.p);
    stream.next(m.v);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
