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
using RotationalInertia_ = KDL::RotationalInertia;

typedef tobas_kdl_msgs::RotationalInertia_<std::allocator<void> > RotationalInertia;

typedef boost::shared_ptr<tobas_kdl_msgs::RotationalInertia> RotationalInertiaPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::RotationalInertia const> RotationalInertiaConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<KDL::RotationalInertia> : TrueType
{
};

template <>
struct IsFixedSize<KDL::RotationalInertia const> : TrueType
{
};

template <>
struct IsMessage<KDL::RotationalInertia> : TrueType
{
};

template <>
struct IsMessage<KDL::RotationalInertia const> : TrueType
{
};

template <>
struct HasHeader<KDL::RotationalInertia> : FalseType
{
};

template <>
struct HasHeader<KDL::RotationalInertia const> : FalseType
{
};

template <>
struct MD5Sum<KDL::RotationalInertia>
{
  static const char* value()
  {
    return "ca66b32e4ad9de837a30ea9fe5ade752";
  }

  static const char* value(const KDL::RotationalInertia&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xca66b32e4ad9de83ULL;
  static const uint64_t static_value2 = 0x7a30ea9fe5ade752ULL;
};

template <>
struct DataType<KDL::RotationalInertia>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/RotationalInertia";
  }

  static const char* value(const KDL::RotationalInertia&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::RotationalInertia>
{
  static const char* value()
  {
    return "# Represents a KDL::RotationalInertia instance.\n\
\n\
float64[9] data\n\
";
  }

  static const char* value(const KDL::RotationalInertia&)
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
struct Serializer<KDL::RotationalInertia>
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
};  // struct RotationalInertia_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<KDL::RotationalInertia>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::RotationalInertia& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < 9; ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // KDL_MSGS_MESSAGE_ROTATIONALINERTIA_H
