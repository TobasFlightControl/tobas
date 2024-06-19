#pragma once

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/wrench.hpp>

#include "./Vector.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using Wrench_ = tobas_kdl::Wrench;

typedef tobas_kdl_msgs::Wrench_<std::allocator<void>> Wrench;
typedef boost::shared_ptr<tobas_kdl_msgs::Wrench> WrenchPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Wrench const> WrenchConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::Wrench> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Wrench const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Wrench> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Wrench const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::Wrench> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::Wrench const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::Wrench>
{
  static const char* value()
  {
    return "4f539cf138b23283b520fd271b567936";
  }

  static const char* value(const tobas_kdl::Wrench&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x4f539cf138b23283ULL;
  static const uint64_t static_value2 = 0xb520fd271b567936ULL;
};

template <>
struct DataType<tobas_kdl::Wrench>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Wrench";
  }

  static const char* value(const tobas_kdl::Wrench&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::Wrench>
{
  static const char* value()
  {
    return "Vector force\n"
           "Vector torque\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_kdl_msgs/Vector\n"
           "float64 x\n"
           "float64 y\n"
           "float64 z\n";
  }

  static const char* value(const tobas_kdl::Wrench&)
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
struct Serializer<tobas_kdl::Wrench>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.force);
    stream.next(m.torque);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
