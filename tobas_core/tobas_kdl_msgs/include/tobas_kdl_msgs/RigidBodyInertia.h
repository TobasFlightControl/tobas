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
using RigidBodyInertia_ = KDL::RigidBodyInertia;

typedef tobas_kdl_msgs::RigidBodyInertia_<std::allocator<void> > RigidBodyInertia;

typedef boost::shared_ptr<tobas_kdl_msgs::RigidBodyInertia> RigidBodyInertiaPtr;
typedef boost::shared_ptr<tobas_kdl_msgs::RigidBodyInertia const> RigidBodyInertiaConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<KDL::RigidBodyInertia> : TrueType
{
};

template <>
struct IsFixedSize<KDL::RigidBodyInertia const> : TrueType
{
};

template <>
struct IsMessage<KDL::RigidBodyInertia> : TrueType
{
};

template <>
struct IsMessage<KDL::RigidBodyInertia const> : TrueType
{
};

template <>
struct HasHeader<KDL::RigidBodyInertia> : FalseType
{
};

template <>
struct HasHeader<KDL::RigidBodyInertia const> : FalseType
{
};

template <>
struct MD5Sum<KDL::RigidBodyInertia>
{
  static const char* value()
  {
    return "33096aa337b8b33a05f701d90acdd33b";
  }

  static const char* value(const KDL::RigidBodyInertia&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x33096aa337b8b33aULL;
  static const uint64_t static_value2 = 0x05f701d90acdd33bULL;
};

template <>
struct DataType<KDL::RigidBodyInertia>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/RigidBodyInertia";
  }

  static const char* value(const KDL::RigidBodyInertia&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::RigidBodyInertia>
{
  static const char* value()
  {
    return "# Represents a KDL::RigidBodyInertia instance.\n\
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
# Represents a KDL::Vector instance.\n\
# This message is compatible to geometry_msgs/Vector3.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: tobas_kdl_msgs/RotationalInertia\n\
# Represents a KDL::RotationalInertia instance.\n\
\n\
float64[9] data\n\
";
  }

  static const char* value(const KDL::RigidBodyInertia&)
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
struct Serializer<KDL::RigidBodyInertia>
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
    typename T::_mass_type _mass;
    stream.next(_mass);
    typename T::_cog_type _cog;
    stream.next(_cog);
    typename T::_Ic_type _Ic;
    stream.next(_Ic);
    m = T(_mass, _cog, _Ic);
  }

  template <typename T>
  inline static uint32_t serializedLength(const T& m)
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
struct Printer<KDL::RigidBodyInertia>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::RigidBodyInertia& v)
  {
    s << indent << "mass: ";
    Printer<double>::stream(s, indent + "  ", v.getMass());
    s << indent << "cog: ";
    s << std::endl;
    Printer<KDL::Vector>::stream(s, indent + "  ", v.getCOG());
    s << indent << "Ic: ";
    s << std::endl;
    Printer<KDL::RotationalInertia>::stream(s, indent + "  ", v.getRotationalInertia());
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // KDL_MSGS_MESSAGE_RIGIDBODYINERTIA_H
