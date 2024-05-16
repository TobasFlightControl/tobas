#ifndef KDL_MSGS_MESSAGE_RIGIDBODYINERTIA_H
#define KDL_MSGS_MESSAGE_RIGIDBODYINERTIA_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/rigidbodyinertia.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using RigidBodyInertia_ = tobas_kdl::RigidBodyInertia;

typedef tobas_kdl_msgs::RigidBodyInertia_<std::allocator<void> > RigidBodyInertia;

typedef boost::shared_ptr<tobas_kdl_msgs::RigidBodyInertia> RigidBodyInertiaPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::RigidBodyInertia const> RigidBodyInertiaConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<tobas_kdl::RigidBodyInertia> : TrueType
{
};

template <>
struct IsFixedSize<tobas_kdl::RigidBodyInertia const> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::RigidBodyInertia> : TrueType
{
};

template <>
struct IsMessage<tobas_kdl::RigidBodyInertia const> : TrueType
{
};

template <>
struct HasHeader<tobas_kdl::RigidBodyInertia> : FalseType
{
};

template <>
struct HasHeader<tobas_kdl::RigidBodyInertia const> : FalseType
{
};

template <>
struct MD5Sum<tobas_kdl::RigidBodyInertia>
{
  static const char* value()
  {
    return "33096aa337b8b33a05f701d90acdd33b";
  }

  static const char* value(const tobas_kdl::RigidBodyInertia&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x33096aa337b8b33aULL;
  static const uint64_t static_value2 = 0x05f701d90acdd33bULL;
};

template <>
struct DataType<tobas_kdl::RigidBodyInertia>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/RigidBodyInertia";
  }

  static const char* value(const tobas_kdl::RigidBodyInertia&)
  {
    return value();
  }
};

template <>
struct Definition<tobas_kdl::RigidBodyInertia>
{
  static const char* value()
  {
    return "# Represents a tobas_kdl::RigidBodyInertia instance.\n\
#\n\
# 6D Inertia of a rigid body\n\
#\n\
\n\
float64 mass\n\
Vector cog\n\
RotationalInertia Ic\n\
\n\
================================================================================\n\
MSG: tobas_kdl_msgs/Vector\n\
# Represents a tobas_kdl::Vector instance.\n\
# This message is compatible to geometry_msgs/Vector3.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: tobas_kdl_msgs/RotationalInertia\n\
# Represents a tobas_kdl::RotationalInertia instance.\n\
\n\
float64[9] data\n\
";
  }

  static const char* value(const tobas_kdl::RigidBodyInertia&)
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
struct Serializer<tobas_kdl::RigidBodyInertia>
{
  template <typename Stream, typename T>
  inline static void write(Stream& stream, const T& m)
  {
    stream.next(m.getMass());
    stream.next(m.getCOG());
    stream.next(m.getRotationalInertia());
  }

  template <typename Stream, typename T>
  inline static void read(Stream& stream, T& m)
  {
    double mass;
    tobas_kdl::Vector cog;
    tobas_kdl::RigidBodyInertia Ic;

    stream.next(mass);
    stream.next(cog);
    stream.next(Ic);

    m = T(mass, cog, Ic);
  }

  template <typename T>
  inline static uint32_t serializedLength(const T&)
  {
    return (uint32_t)(sizeof(double) + 3 * sizeof(double) + 9 * sizeof(double));
  }
};  // struct RigidBodyInertia_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<tobas_kdl::RigidBodyInertia>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const tobas_kdl::RigidBodyInertia& v)
  {
    s << indent << "mass: ";
    Printer<double>::stream(s, indent + "  ", v.getMass());
    s << indent << "cog: ";
    s << std::endl;
    Printer<tobas_kdl::Vector>::stream(s, indent + "  ", v.getCOG());
    s << indent << "Ic: ";
    s << std::endl;
    Printer<tobas_kdl::RotationalInertia>::stream(s, indent + "  ", v.getRotationalInertia());
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // KDL_MSGS_MESSAGE_RIGIDBODYINERTIA_H
