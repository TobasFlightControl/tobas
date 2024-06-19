#ifndef KDL_MSGS_MESSAGE_ROTATIONALINERTIA_H
#define KDL_MSGS_MESSAGE_ROTATIONALINERTIA_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/rotationalinertia.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using RotationalInertia_ = tobas_kdl::RotationalInertia;

typedef tobas_kdl_msgs::RotationalInertia_<std::allocator<void>> RotationalInertia;
typedef boost::shared_ptr<tobas_kdl_msgs::RotationalInertia> RotationalInertiaPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::RotationalInertia const> RotationalInertiaConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<tobas_kdl::RotationalInertia> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::RotationalInertia const> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::RotationalInertia> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::RotationalInertia const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::RotationalInertia> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::RotationalInertia const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::RotationalInertia>
{
  static const char* value()
  {
    return "ca66b32e4ad9de837a30ea9fe5ade752";
  }

  static const char* value(const tobas_kdl::RotationalInertia&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xca66b32e4ad9de83ULL;
  static const uint64_t static_value2 = 0x7a30ea9fe5ade752ULL;
};

template <>
struct DataType<tobas_kdl::RotationalInertia>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/RotationalInertia";
  }

  static const char* value(const tobas_kdl::RotationalInertia&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::RotationalInertia>
{
  static const char* value()
  {
    return "# Represents a tobas_kdl::RotationalInertia instance.\n\
\n\
float64[9] data\n\
";
  }

  static const char* value(const tobas_kdl::RotationalInertia&)
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
struct Serializer<tobas_kdl::RotationalInertia>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    for (size_t row = 0; row < 3; ++row)
      for (size_t col = 0; col < 3; ++col)
        stream.next(m.data(row, col));
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros

#endif  // KDL_MSGS_MESSAGE_ROTATIONALINERTIA_H
