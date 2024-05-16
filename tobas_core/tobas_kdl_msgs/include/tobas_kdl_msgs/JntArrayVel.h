#ifndef TOBAS_KDL_MSGS_MESSAGE_JNTARRAYVEL_H
#define TOBAS_KDL_MSGS_MESSAGE_JNTARRAYVEL_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/jntarrayvel.hpp>

#include "./JntArray.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using JntArrayVel_ = tobas_kdl::JntArrayVel;

typedef tobas_kdl_msgs::JntArrayVel_<std::allocator<void>> JntArrayVel;

typedef boost::shared_ptr<tobas_kdl_msgs::JntArrayVel> JntArrayVelPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::JntArrayVel const> JntArrayVelConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::JntArrayVel> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::JntArrayVel const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::JntArrayVel> : FalseType
{
};

template <>
struct IsFixedSize<tobas_kdl::JntArrayVel const> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::JntArrayVel> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::JntArrayVel const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::JntArrayVel>
{
  static const char* value()
  {
    return "45a5c905c9481a71e7b5dee770e487ce";
  }

  static const char* value(const tobas_kdl::JntArrayVel&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x45a5c905c9481a71ULL;
  static const uint64_t static_value2 = 0xe7b5dee770e487ceULL;
};

template <>
struct DataType<tobas_kdl::JntArrayVel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/JntArrayVel";
  }

  static const char* value(const tobas_kdl::JntArrayVel&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::JntArrayVel>
{
  static const char* value()
  {
    return "float64[] q\n"
           "float64[] qdot\n";
  }

  static const char* value(const tobas_kdl::JntArrayVel&)
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
struct Serializer<tobas_kdl::JntArrayVel>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.q);
    stream.next(m.qdot);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct JntArrayVel_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<tobas_kdl::JntArrayVel>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const tobas_kdl::JntArrayVel& v)
  {
    s << indent << "q[]" << std::endl;
    for (size_t i = 0; i < v.q.size(); ++i)
    {
      s << indent << "  q[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.q(i));
    }
    s << indent << "qdot[]" << std::endl;
    for (size_t i = 0; i < v.qdot.size(); ++i)
    {
      s << indent << "  qdot[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.qdot(i));
    }
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_JNTARRAYVEL_H
