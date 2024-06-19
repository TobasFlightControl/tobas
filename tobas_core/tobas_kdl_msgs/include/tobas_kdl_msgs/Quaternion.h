#pragma once

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/quaternion.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Quaternion_ = tobas_kdl::Quaternion;

typedef tobas_kdl_msgs::Quaternion_<std::allocator<void>> Quaternion;
typedef boost::shared_ptr<tobas_kdl_msgs::Quaternion> QuaternionPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Quaternion const> QuaternionConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::Quaternion> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Quaternion const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Quaternion> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Quaternion const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::Quaternion> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::Quaternion const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::Quaternion>
{
  static const char* value()
  {
    return "a779879fadf0160734f906b8c19c7004";
  }

  static const char* value(const tobas_kdl::Quaternion&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xa779879fadf01607ULL;
  static const uint64_t static_value2 = 0x34f906b8c19c7004ULL;
};

template <>
struct DataType<tobas_kdl::Quaternion>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Quaternion";
  }

  static const char* value(const tobas_kdl::Quaternion&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::Quaternion>
{
  static const char* value()
  {
    return "float64 x\n"
           "float64 y\n"
           "float64 z\n"
           "float64 w\n";
  }

  static const char* value(const tobas_kdl::Quaternion&)
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
