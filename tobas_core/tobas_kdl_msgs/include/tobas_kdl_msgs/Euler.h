#pragma once

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/euler.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Euler_ = tobas_kdl::Euler;

typedef tobas_kdl_msgs::Euler_<std::allocator<void>> Euler;
typedef boost::shared_ptr<tobas_kdl_msgs::Euler> EulerPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Euler const> EulerConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::Euler> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Euler const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Euler> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Euler const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::Euler> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::Euler const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::Euler>
{
  static const char* value()
  {
    return "eeec8b25a660789a89540dedcb2b06d6";
  }

  static const char* value(const tobas_kdl::Euler&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xeeec8b25a660789aULL;
  static const uint64_t static_value2 = 0x89540dedcb2b06d6ULL;
};

template <>
struct DataType<tobas_kdl::Euler>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Euler";
  }

  static const char* value(const tobas_kdl::Euler&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::Euler>
{
  static const char* value()
  {
    return "float64 roll   # [rad]\n"
           "float64 pitch  # [rad] \n"
           "float64 yaw    # [rad]\n";
  }

  static const char* value(const tobas_kdl::Euler&)
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
struct Serializer<tobas_kdl::Euler>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
