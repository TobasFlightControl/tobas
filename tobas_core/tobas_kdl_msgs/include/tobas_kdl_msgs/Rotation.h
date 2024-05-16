#ifndef KDL_MSGS_MESSAGE_ROTATION_H
#define KDL_MSGS_MESSAGE_ROTATION_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/rotation.hpp>

namespace tobas_kdl_msgs
{
// using C++11 syntax KDL::Rotation and tobas_kdl_msgs::Rotation_ are exactly the same type
template <typename ContainerAllocator>
using Rotation_ = KDL::Rotation;

typedef tobas_kdl_msgs::Rotation_<std::allocator<void> > Rotation;

typedef boost::shared_ptr<tobas_kdl_msgs::Rotation> RotationPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Rotation const> RotationConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<KDL::Rotation> : TrueType
{
};

template <>
struct IsFixedSize<KDL::Rotation const> : TrueType
{
};

template <>
struct IsMessage<KDL::Rotation> : TrueType
{
};

template <>
struct IsMessage<KDL::Rotation const> : TrueType
{
};

template <>
struct HasHeader<KDL::Rotation> : FalseType
{
};

template <>
struct HasHeader<KDL::Rotation const> : FalseType
{
};

template <>
struct MD5Sum<KDL::Rotation>
{
  static const char* value()
  {
    return "ca66b32e4ad9de837a30ea9fe5ade752";
  }

  static const char* value(const KDL::Rotation&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xca66b32e4ad9de83ULL;
  static const uint64_t static_value2 = 0x7a30ea9fe5ade752ULL;
};

template <>
struct DataType<KDL::Rotation>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Rotation";
  }

  static const char* value(const KDL::Rotation&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::Rotation>
{
  static const char* value()
  {
    return "# Represents a KDL::Rotation instance.\n\
\n\
float64[9] data\n\
";
  }

  static const char* value(const KDL::Rotation&)
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
struct Serializer<KDL::Rotation>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data(0, 0));
    stream.next(m.data(0, 1));
    stream.next(m.data(0, 2));
    stream.next(m.data(1, 0));
    stream.next(m.data(1, 1));
    stream.next(m.data(1, 2));
    stream.next(m.data(2, 0));
    stream.next(m.data(2, 1));
    stream.next(m.data(2, 2));
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct Rotation_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<KDL::Rotation>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::Rotation& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < 3; ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        s << indent << "  data[" << i << "]: ";
        Printer<double>::stream(s, indent + "  ", v.data(i, j));
      }
    }
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // KDL_MSGS_MESSAGE_ROTATION_H
