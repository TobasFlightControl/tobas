#pragma once

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/jntarrayacc.hpp>

#include "./JntArray.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using JntArrayAcc_ = tobas_kdl::JntArrayAcc;

typedef tobas_kdl_msgs::JntArrayAcc_<std::allocator<void>> JntArrayAcc;
typedef boost::shared_ptr<tobas_kdl_msgs::JntArrayAcc> JntArrayAccPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::JntArrayAcc const> JntArrayAccConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::JntArrayAcc> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::JntArrayAcc const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::JntArrayAcc> : FalseType
{
};

template <>
struct IsFixedSize<tobas_kdl::JntArrayAcc const> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::JntArrayAcc> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::JntArrayAcc const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::JntArrayAcc>
{
  static const char* value()
  {
    return "cb75d1f3d3b3f8137a70dc309f21917e";
  }

  static const char* value(const tobas_kdl::JntArrayAcc&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xcb75d1f3d3b3f813ULL;
  static const uint64_t static_value2 = 0x7a70dc309f21917eULL;
};

template <>
struct DataType<tobas_kdl::JntArrayAcc>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/JntArrayAcc";
  }

  static const char* value(const tobas_kdl::JntArrayAcc&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::JntArrayAcc>
{
  static const char* value()
  {
    return "float64[] q\n"
           "float64[] qdot\n"
           "float64[] qdotdot\n";
  }

  static const char* value(const tobas_kdl::JntArrayAcc&)
  {
    return value();
  }
};

}  // namespace message_traits
}  // namespace ros

namespace ros
{
namespace serialization
{
template <>
struct Serializer<tobas_kdl::JntArrayAcc>
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
