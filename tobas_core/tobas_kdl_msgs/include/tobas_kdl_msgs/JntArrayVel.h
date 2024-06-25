#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/jntarrayvel.hpp>

#include "./JntArray.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using JntArrayVel_ = kdl::JntArrayVel;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<kdl::JntArrayVel>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.q);
    stream.next(m.qdot);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
