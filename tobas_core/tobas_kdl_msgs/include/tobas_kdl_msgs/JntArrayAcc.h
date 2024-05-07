#ifndef TOBAS_KDL_MSGS_MESSAGE_JNTARRAYACC_H
#define TOBAS_KDL_MSGS_MESSAGE_JNTARRAYACC_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/jntarrayacc.hpp>

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using JntArrayAcc_ = KDL::JntArrayAcc;

typedef tobas_kdl_msgs::JntArrayAcc_<std::allocator<void>> JntArrayAcc;

typedef boost::shared_ptr<tobas_kdl_msgs::JntArrayAcc> JntArrayAccPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::JntArrayAcc const> JntArrayAccConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<KDL::JntArrayAcc> : TrueType
{
};

template <>
struct IsMessage<KDL::JntArrayAcc const> : TrueType
{
};

template <>
struct IsFixedSize<KDL::JntArrayAcc> : FalseType
{
};

template <>
struct IsFixedSize<KDL::JntArrayAcc const> : FalseType
{
};

template <>
struct HasHeader<KDL::JntArrayAcc> : FalseType
{
};

template <>
struct HasHeader<KDL::JntArrayAcc const> : FalseType
{
};

template <>
struct MD5Sum<KDL::JntArrayAcc>
{
  static const char* value()
  {
    return "cb75d1f3d3b3f8137a70dc309f21917e";
  }

  static const char* value(const KDL::JntArrayAcc&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xcb75d1f3d3b3f813ULL;
  static const uint64_t static_value2 = 0x7a70dc309f21917eULL;
};

template <>
struct DataType<KDL::JntArrayAcc>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/JntArrayAcc";
  }

  static const char* value(const KDL::JntArrayAcc&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::JntArrayAcc>
{
  static const char* value()
  {
    return "float64[] q\n"
           "float64[] qdot\n"
           "float64[] qdotdot\n";
  }

  static const char* value(const KDL::JntArrayAcc&)
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
struct Serializer<KDL::JntArrayAcc>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.q);
    stream.next(m.qdot);
    stream.next(m.qdotdot);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct JntArrayAcc_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<KDL::JntArrayAcc>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::JntArrayAcc& v)
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
    s << indent << "qdotdot[]" << std::endl;
    for (size_t i = 0; i < v.qdotdot.size(); ++i)
    {
      s << indent << "  qdotdot[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.qdotdot[i]);
    }
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_JNTARRAYACC_H
