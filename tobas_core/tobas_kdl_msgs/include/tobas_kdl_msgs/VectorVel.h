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
using VectorVel_ = KDL::VectorVel;

typedef tobas_kdl_msgs::VectorVel_<std::allocator<void> > VectorVel;

typedef boost::shared_ptr<tobas_kdl_msgs::VectorVel> VectorVelPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::VectorVel const> VectorVelConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<KDL::VectorVel> : TrueType
{
};

template <>
struct IsMessage<KDL::VectorVel const> : TrueType
{
};

template <>
struct IsFixedSize<KDL::VectorVel> : TrueType
{
};

template <>
struct IsFixedSize<KDL::VectorVel const> : TrueType
{
};

template <>
struct HasHeader<KDL::VectorVel> : FalseType
{
};

template <>
struct HasHeader<KDL::VectorVel const> : FalseType
{
};

template <>
struct MD5Sum<KDL::VectorVel>
{
  static const char* value()
  {
    return "5596953d851bb0bf15ba1b09a4a1c3c1";
  }

  static const char* value(const KDL::VectorVel&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x5596953d851bb0bfULL;
  static const uint64_t static_value2 = 0x15ba1b09a4a1c3c1ULL;
};

template <>
struct DataType<KDL::VectorVel>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/VectorVel";
  }

  static const char* value(const KDL::VectorVel&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::VectorVel>
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

  static const char* value(const KDL::VectorVel&)
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
struct Serializer<KDL::VectorVel>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.p);
    stream.next(m.v);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct VectorVel_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<KDL::VectorVel>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::VectorVel& v)
  {
    s << indent << "p: ";
    s << std::endl;
    Printer<KDL::Vector>::stream(s, indent + "  ", v.p);
    s << indent << "v: ";
    s << std::endl;
    Printer<KDL::Vector>::stream(s, indent + "  ", v.v);
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_VECTORVEL_H
