#ifndef TOBAS_KDL_MSGS_MESSAGE_VECTORVEL_H
#define TOBAS_KDL_MSGS_MESSAGE_VECTORVEL_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/vectorvel.hpp>

#include "./Vector.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using VectorVel_ = tobas_kdl::VectorVel;

typedef tobas_kdl_msgs::VectorVel_<std::allocator<void>> VectorVel;
typedef boost::shared_ptr<tobas_kdl_msgs::VectorVel> VectorVelPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::VectorVel const> VectorVelConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<tobas_kdl::VectorVel> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::VectorVel const> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::VectorVel> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::VectorVel const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::VectorVel> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::VectorVel const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::VectorVel>
{
  static const char* value()
  {
    return "5596953d851bb0bf15ba1b09a4a1c3c1";
  }

  static const char* value(const tobas_kdl::VectorVel&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x5596953d851bb0bfULL;
  static const uint64_t static_value2 = 0x15ba1b09a4a1c3c1ULL;
};

template <>
struct DataType<tobas_kdl::VectorVel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/VectorVel";
  }

  static const char* value(const tobas_kdl::VectorVel&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::VectorVel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/Vector p\n"
           "tobas_kdl_msgs/Vector v\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_kdl_msgs/Vector\n"
           "float64 x\n"
           "float64 y\n"
           "float64 z\n";
  }

  static const char* value(const tobas_kdl::VectorVel&)
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
struct Serializer<tobas_kdl::VectorVel>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.p);
    stream.next(m.v);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_VECTORVEL_H
