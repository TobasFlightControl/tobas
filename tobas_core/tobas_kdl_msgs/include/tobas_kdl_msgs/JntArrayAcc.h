#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/jntarrayacc.hpp>

#include "./JntArray.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using JntArrayAcc_ = kdl::JntArrayAcc;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<kdl::JntArrayAcc>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.q);
    stream.next(m.qdot);
    stream.next(m.qdotdot);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
