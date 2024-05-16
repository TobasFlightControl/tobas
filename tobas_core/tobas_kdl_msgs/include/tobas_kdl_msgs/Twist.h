#ifndef KDL_MSGS_MESSAGE_TWIST_H
#define KDL_MSGS_MESSAGE_TWIST_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/twist.hpp>

#include "./Vector.h"

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Twist_ = tobas_kdl::Twist;

typedef tobas_kdl_msgs::Twist_<std::allocator<void> > Twist;

typedef boost::shared_ptr<tobas_kdl_msgs::Twist> TwistPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Twist const> TwistConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<tobas_kdl::Twist> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Twist const> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Twist> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Twist const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::Twist> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::Twist const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::Twist>
{
  static const char* value()
  {
    return "9f195f881246fdfa2798d1d3eebca84a";
  }

  static const char* value(const tobas_kdl::Twist&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x9f195f881246fdfaULL;
  static const uint64_t static_value2 = 0x2798d1d3eebca84aULL;
};

template <>
struct DataType<tobas_kdl::Twist>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Twist";
  }

  static const char* value(const tobas_kdl::Twist&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::Twist>
{
  static const char* value()
  {
    return "# Represents a tobas_kdl::Twist instance.\n\
# This message is compatible to geometry_msgs/Twist.\n\
\n\
Vector linear\n\
Vector angular\n\
\n\
================================================================================\n\
MSG: tobas_kdl_msgs/Vector\n\
# Represents a tobas_kdl::Vector instance.\n\
# This message is compatible to geometry_msgs/Vector3.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const tobas_kdl::Twist&)
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
struct Serializer<tobas_kdl::Twist>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.vel);
    stream.next(m.rot);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct Twist_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<tobas_kdl::Twist>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const tobas_kdl::Twist& v)
  {
    s << indent << "linear: ";
    s << std::endl;
    Printer<tobas_kdl::Vector>::stream(s, indent + "  ", v.vel);
    s << indent << "angular: ";
    s << std::endl;
    Printer<tobas_kdl::Vector>::stream(s, indent + "  ", v.rot);
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // KDL_MSGS_MESSAGE_TWIST_H
