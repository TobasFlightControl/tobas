#ifndef KDL_MSGS_MESSAGE_VECTOR_H
#define KDL_MSGS_MESSAGE_VECTOR_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/vector.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Vector_ = tobas_kdl::Vector;

typedef tobas_kdl_msgs::Vector_<std::allocator<void> > Vector;

typedef boost::shared_ptr<tobas_kdl_msgs::Vector> VectorPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Vector const> VectorConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<tobas_kdl::Vector> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Vector const> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Vector> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Vector const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::Vector> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::Vector const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::Vector>
{
  static const char* value()
  {
    return "4a842b65f413084dc2b10fb484ea7f17";
  }

  static const char* value(const tobas_kdl::Vector&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x4a842b65f413084dULL;
  static const uint64_t static_value2 = 0xc2b10fb484ea7f17ULL;
};

template <>
struct DataType<tobas_kdl::Vector>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Vector";
  }

  static const char* value(const tobas_kdl::Vector&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::Vector>
{
  static const char* value()
  {
    return "# Represents a tobas_kdl::Vector instance.\n\
# This message is compatible to geometry_msgs/Vector3.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const tobas_kdl::Vector&)
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
struct Serializer<tobas_kdl::Vector>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data.x());
    stream.next(m.data.y());
    stream.next(m.data.z());
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct Vector_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<tobas_kdl::Vector>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const tobas_kdl::Vector& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.data.x());
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.data.y());
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.data.z());
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // KDL_MSGS_MESSAGE_VECTOR_H
