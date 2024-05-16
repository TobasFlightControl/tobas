#ifndef TOBAS_KDL_MSGS_MESSAGE_FRAMEVEL_H
#define TOBAS_KDL_MSGS_MESSAGE_FRAMEVEL_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/framevel.hpp>

#include "./Frame.h"
#include "./Twist.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using FrameVel_ = tobas_kdl::FrameVel;

typedef tobas_kdl_msgs::FrameVel_<std::allocator<void> > FrameVel;

typedef boost::shared_ptr<tobas_kdl_msgs::FrameVel> FrameVelPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::FrameVel const> FrameVelConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::FrameVel> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::FrameVel const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::FrameVel> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::FrameVel const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::FrameVel> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::FrameVel const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::FrameVel>
{
  static const char* value()
  {
    return "2a4ef0eeda46a6fb602f53c28f12ffef";
  }

  static const char* value(const tobas_kdl::FrameVel&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x2a4ef0eeda46a6fbULL;
  static const uint64_t static_value2 = 0x602f53c28f12ffefULL;
};

template <>
struct DataType<tobas_kdl::FrameVel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/FrameVel";
  }

  static const char* value(const tobas_kdl::FrameVel&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::FrameVel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Frame frame\n"
           "tobas_kdl_msgs/Twist twist\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_kdl_msgs/Frame\n"
           "tobas_kdl_msgs/Vector trans\n"
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
           "float64[9] data\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_kdl_msgs/Twist\n"
           "tobas_kdl_msgs/Vector linear\n"
           "tobas_kdl_msgs/Vector angular\n";
  }

  static const char* value(const tobas_kdl::FrameVel&)
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
struct Serializer<tobas_kdl::FrameVel>
{
  template <typename Stream, typename T>
  inline static void write(Stream& stream, const T& m)
  {
    stream.next(m.getFrame());
    stream.next(m.getTwist());
  }

  template <typename Stream, typename T>
  inline static void read(Stream& stream, T& m)
  {
    tobas_kdl::Frame frame;
    tobas_kdl::Twist twist;

    stream.next(frame);
    stream.next(twist);

    m = T(frame, twist);
  }

  template <typename T>
  inline static uint32_t serializedLength(const T&)
  {
    return (uint32_t)(3 * sizeof(double) + 3 * sizeof(double));
  }
};  // struct FrameVel_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<tobas_kdl::FrameVel>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const tobas_kdl::FrameVel& v)
  {
    s << indent << "frame: ";
    s << std::endl;
    Printer<tobas_kdl::Frame>::stream(s, indent + "  ", v.getFrame());
    s << indent << "twist: ";
    s << std::endl;
    Printer<tobas_kdl::Twist>::stream(s, indent + "  ", v.getTwist());
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_FRAMEVEL_H
