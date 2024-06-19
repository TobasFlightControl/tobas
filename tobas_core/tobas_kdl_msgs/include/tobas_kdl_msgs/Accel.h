#pragma once

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/accel.hpp>

#include "./Vector.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using Accel_ = tobas_kdl::Accel;

typedef tobas_kdl_msgs::Accel_<std::allocator<void>> Accel;
typedef boost::shared_ptr<tobas_kdl_msgs::Accel> AccelPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Accel const> AccelConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::Accel> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Accel const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Accel> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Accel const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::Accel> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::Accel const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::Accel>
{
  static const char* value()
  {
    return "9f195f881246fdfa2798d1d3eebca84a";
  }

  static const char* value(const tobas_kdl::Accel&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x9f195f881246fdfaULL;
  static const uint64_t static_value2 = 0x2798d1d3eebca84aULL;
};

template <>
struct DataType<tobas_kdl::Accel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Accel";
  }

  static const char* value(const tobas_kdl::Accel&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::Accel>
{
  static const char* value()
  {
    return "Vector linear\n"
           "Vector angular\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_kdl_msgs/Vector\n"
           "float64 x\n"
           "float64 y\n"
           "float64 z\n";
  }

  static const char* value(const tobas_kdl::Accel&)
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
struct Serializer<tobas_kdl::Accel>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.linear);
    stream.next(m.angular);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
