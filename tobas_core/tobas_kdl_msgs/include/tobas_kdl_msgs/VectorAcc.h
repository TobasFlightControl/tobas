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
using VectorAcc_ = tobas_kdl::VectorAcc;

typedef tobas_kdl_msgs::VectorAcc_<std::allocator<void>> VectorAcc;
typedef boost::shared_ptr<tobas_kdl_msgs::VectorAcc> VectorAccPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::VectorAcc const> VectorAccConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::VectorAcc> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::VectorAcc const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::VectorAcc> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::VectorAcc const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::VectorAcc> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::VectorAcc const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::VectorAcc>
{
  static const char* value()
  {
    return "25990730987cb33c6804001eca036b26";
  }

  static const char* value(const tobas_kdl::VectorAcc&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x25990730987cb33cULL;
  static const uint64_t static_value2 = 0x6804001eca036b26ULL;
};

template <>
struct DataType<tobas_kdl::VectorAcc>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/VectorAcc";
  }

  static const char* value(const tobas_kdl::VectorAcc&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::VectorAcc>
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

  static const char* value(const tobas_kdl::VectorAcc&)
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
struct Serializer<tobas_kdl::VectorAcc>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.p);
    stream.next(m.v);
    stream.next(m.dv);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_VECTORACC_H
