#ifndef TOBAS_KDL_MSGS_MESSAGE_ACCEL_H
#define TOBAS_KDL_MSGS_MESSAGE_ACCEL_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/accel.hpp>

#include "./Vector.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using Accel_ = KDL::Accel;

typedef tobas_kdl_msgs::Accel_<std::allocator<void> > Accel;

typedef boost::shared_ptr<tobas_kdl_msgs::Accel> AccelPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Accel const> AccelConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<KDL::Accel> : TrueType
{
};

template <>
struct IsMessage<KDL::Accel const> : TrueType
{
};

template <>
struct IsFixedSize<KDL::Accel> : TrueType
{
};

template <>
struct IsFixedSize<KDL::Accel const> : TrueType
{
};

template <>
struct HasHeader<KDL::Accel> : FalseType
{
};

template <>
struct HasHeader<KDL::Accel const> : FalseType
{
};

template <>
struct MD5Sum<KDL::Accel>
{
  static const char* value()
  {
    return "9f195f881246fdfa2798d1d3eebca84a";
  }

  static const char* value(const KDL::Accel&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x9f195f881246fdfaULL;
  static const uint64_t static_value2 = 0x2798d1d3eebca84aULL;
};

template <>
struct DataType<KDL::Accel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Accel";
  }

  static const char* value(const KDL::Accel&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::Accel>
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

  static const char* value(const KDL::Accel&)
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
struct Serializer<KDL::Accel>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.linear);
    stream.next(m.angular);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct Accel_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<KDL::Accel>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::Accel& v)
  {
    s << indent << "linear: ";
    s << std::endl;
    Printer<KDL::Vector>::stream(s, indent + "  ", v.linear);
    s << indent << "angular: ";
    s << std::endl;
    Printer<KDL::Vector>::stream(s, indent + "  ", v.angular);
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_ACCEL_H
