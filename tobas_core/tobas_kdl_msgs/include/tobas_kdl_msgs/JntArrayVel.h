#ifndef TOBAS_KDL_MSGS_MESSAGE_JNTARRAYVEL_H
#define TOBAS_KDL_MSGS_MESSAGE_JNTARRAYVEL_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/jntarrayvel.hpp>

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using JntArrayVel_ = KDL::JntArrayVel;

typedef ::tobas_kdl_msgs::JntArrayVel_<std::allocator<void>> JntArrayVel;

typedef boost::shared_ptr<::tobas_kdl_msgs::JntArrayVel> JntArrayVelPtr;
typedef boost::shared_ptr<::tobas_kdl_msgs::JntArrayVel const> JntArrayVelConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<KDL::JntArrayVel> : TrueType
{
};

template <>
struct IsMessage<KDL::JntArrayVel const> : TrueType
{
};

template <>
struct IsFixedSize<KDL::JntArrayVel> : FalseType
{
};

template <>
struct IsFixedSize<KDL::JntArrayVel const> : FalseType
{
};

template <>
struct HasHeader<KDL::JntArrayVel> : FalseType
{
};

template <>
struct HasHeader<KDL::JntArrayVel const> : FalseType
{
};

template <>
struct MD5Sum<KDL::JntArrayVel>
{
  static const char* value()
  {
    return "45a5c905c9481a71e7b5dee770e487ce";
  }

  static const char* value(const KDL::JntArrayVel&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x45a5c905c9481a71ULL;
  static const uint64_t static_value2 = 0xe7b5dee770e487ceULL;
};

template <>
struct DataType<KDL::JntArrayVel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/JntArrayVel";
  }

  static const char* value(const KDL::JntArrayVel&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::JntArrayVel>
{
  static const char* value()
  {
    return "float64[] q\n"
           "float64[] qdot\n";
  }

  static const char* value(const KDL::JntArrayVel&)
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
struct Serializer<KDL::JntArrayVel>
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
struct Printer<KDL::JntArrayVel>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::JntArrayVel& v)
  {
    s << indent << "q[]" << std::endl;
    for (size_t i = 0; i < v.q.size(); ++i)
    {
      s << indent << "  q[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.q[i]);
    }
    s << indent << "qdot[]" << std::endl;
    for (size_t i = 0; i < v.qdot.size(); ++i)
    {
      s << indent << "  qdot[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.qdot[i]);
    }
  }
};

}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_JNTARRAYVEL_H
