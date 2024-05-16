#ifndef TOBAS_KDL_MSGS_MESSAGE_FRAME_H
#define TOBAS_KDL_MSGS_MESSAGE_FRAME_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/frame.hpp>

#include "./Vector.h"
#include "./Rotation.h"

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using Frame_ = tobas_kdl::Frame;

typedef tobas_kdl_msgs::Frame_<std::allocator<void> > Frame;

typedef boost::shared_ptr<tobas_kdl_msgs::Frame> FramePtr;
typedef boost::shared_ptr<tobas_kdl_msgs::Frame const> FrameConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::Frame> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::Frame const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Frame> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::Frame const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::Frame> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::Frame const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::Frame>
{
  static const char* value()
  {
    return "0aee6f06cea54f3cb5357fdf456e198c";
  }

  static const char* value(const tobas_kdl::Frame&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x0aee6f06cea54f3cULL;
  static const uint64_t static_value2 = 0xb5357fdf456e198cULL;
};

template <>
struct DataType<tobas_kdl::Frame>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Frame";
  }

  static const char* value(const tobas_kdl::Frame&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::Frame>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Vector trans\n"
           "tobas_kdl_msgs/Rotation rot\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_kdl_msgs/Vector\n"
           "float64 x\n"
           "float64 y\n"
           "float64 z\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_kdl_msgs/Rotation\n"
           "float64[9] data\n";
  }

  static const char* value(const tobas_kdl::Frame&)
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
struct Serializer<tobas_kdl::Frame>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.p);
    stream.next(m.M);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct Frame_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<tobas_kdl::Frame>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const tobas_kdl::Frame& v)
  {
    s << indent << "trans: ";
    s << std::endl;
    Printer<tobas_kdl::Vector>::stream(s, indent + "  ", v.p);
    s << indent << "rot: ";
    s << std::endl;
    Printer<tobas_kdl::Rotation>::stream(s, indent + "  ", v.M);
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_FRAME_H
