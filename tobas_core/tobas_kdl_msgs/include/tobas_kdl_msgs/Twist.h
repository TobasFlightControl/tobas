#ifndef KDL_MSGS_MESSAGE_TWIST_H
#define KDL_MSGS_MESSAGE_TWIST_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/twist.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Twist_ = KDL::Twist;

typedef ::tobas_kdl_msgs::Twist_<std::allocator<void> > Twist;

typedef boost::shared_ptr< ::tobas_kdl_msgs::Twist> TwistPtr;
typedef boost::shared_ptr< ::tobas_kdl_msgs::Twist const> TwistConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<KDL::Twist> : TrueType
{
};

template <>
struct IsFixedSize<KDL::Twist const> : TrueType
{
};

template <>
struct IsMessage<KDL::Twist> : TrueType
{
};

template <>
struct IsMessage<KDL::Twist const> : TrueType
{
};

template <>
struct HasHeader<KDL::Twist> : FalseType
{
};

template <>
struct HasHeader<KDL::Twist const> : FalseType
{
};

template <>
struct MD5Sum<KDL::Twist>
{
  static const char* value()
  {
    return "9f195f881246fdfa2798d1d3eebca84a";
  }

  static const char* value(const KDL::Twist&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x9f195f881246fdfaULL;
  static const uint64_t static_value2 = 0x2798d1d3eebca84aULL;
};

template <>
struct DataType<KDL::Twist>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Twist";
  }

  static const char* value(const KDL::Twist&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::Twist>
{
  static const char* value()
  {
    return "# Represents a KDL::Twist instance.\n\
# This message is compatible to geometry_msgs/Twist.\n\
\n\
Vector linear\n\
Vector angular\n\
\n\
================================================================================\n\
MSG: tobas_kdl_msgs/Vector\n\
# Represents a KDL::Vector instance.\n\
# This message is compatible to geometry_msgs/Vector3.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const KDL::Twist&)
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
struct Serializer<KDL::Twist>
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
struct Printer<KDL::Twist>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::Twist& v)
  {
    s << indent << "linear: ";
    s << std::endl;
    Printer<KDL::Vector>::stream(s, indent + "  ", v.vel);
    s << indent << "angular: ";
    s << std::endl;
    Printer<KDL::Vector>::stream(s, indent + "  ", v.rot);
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // KDL_MSGS_MESSAGE_TWIST_H
