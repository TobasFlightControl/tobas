#ifndef TOBAS_KDL_MSGS_MESSAGE_VECTORACC_H
#define TOBAS_KDL_MSGS_MESSAGE_VECTORACC_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/vectoracc.hpp>

#include "./Vector.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using VectorAcc_ = KDL::VectorAcc;

typedef tobas_kdl_msgs::VectorAcc_<std::allocator<void> > VectorAcc;

typedef boost::shared_ptr<tobas_kdl_msgs::VectorAcc> VectorAccPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::VectorAcc const> VectorAccConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<KDL::VectorAcc> : TrueType
{
};

template <>
struct IsMessage<KDL::VectorAcc const> : TrueType
{
};

template <>
struct IsFixedSize<KDL::VectorAcc> : TrueType
{
};

template <>
struct IsFixedSize<KDL::VectorAcc const> : TrueType
{
};

template <>
struct HasHeader<KDL::VectorAcc> : FalseType
{
};

template <>
struct HasHeader<KDL::VectorAcc const> : FalseType
{
};

template <>
struct MD5Sum<KDL::VectorAcc>
{
  static const char* value()
  {
    return "25990730987cb33c6804001eca036b26";
  }

  static const char* value(const KDL::VectorAcc&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x25990730987cb33cULL;
  static const uint64_t static_value2 = 0x6804001eca036b26ULL;
};

template <>
struct DataType<KDL::VectorAcc>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/VectorAcc";
  }

  static const char* value(const KDL::VectorAcc&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::VectorAcc>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Vector p\n"
           "tobas_kdl_msgs/Vector v\n"
           "tobas_kdl_msgs/Vector dv\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_kdl_msgs/Vector\n"
           "float64 x\n"
           "float64 y\n"
           "float64 z\n";
  }

  static const char* value(const KDL::VectorAcc&)
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
struct Serializer<KDL::VectorAcc>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.p);
    stream.next(m.v);
    stream.next(m.dv);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct VectorAcc_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<KDL::VectorAcc>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::VectorAcc& v)
  {
    s << indent << "p: ";
    s << std::endl;
    Printer<tobas_kdl_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.p);
    s << indent << "v: ";
    s << std::endl;
    Printer<tobas_kdl_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.v);
    s << indent << "dv: ";
    s << std::endl;
    Printer<tobas_kdl_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.dv);
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_VECTORACC_H
