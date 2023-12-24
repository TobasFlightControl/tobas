#ifndef TOBAS_MSGS_MESSAGE_EULER_H
#define TOBAS_MSGS_MESSAGE_EULER_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <dh_kdl/euler.hpp>  // Added

namespace dh_kdl_msgs
{
/* Added from here */
template <typename ContainerAllocator>
using Euler_ = KDL::Euler;
/* Added to here */

/* Commented out from here
template <>
struct Euler_
{
  typedef Euler_<ContainerAllocator> Type;

  Euler_() : roll(0.0), pitch(0.0), yaw(0.0)
  {
  }
  Euler_(const ContainerAllocator& _alloc) : roll(0.0), pitch(0.0), yaw(0.0)
  {
    (void)_alloc;
  }

  typedef double _roll_type;
  _roll_type roll;

  typedef double _pitch_type;
  _pitch_type pitch;

  typedef double _yaw_type;
  _yaw_type yaw;

  typedef boost::shared_ptr< KDL::Euler > Ptr;
  typedef boost::shared_ptr< KDL::Euler const> ConstPtr;

};  // struct Euler_
Commented out to here */

typedef ::dh_kdl_msgs::Euler_<std::allocator<void> > Euler;

typedef boost::shared_ptr< ::dh_kdl_msgs::Euler> EulerPtr;
typedef boost::shared_ptr< ::dh_kdl_msgs::Euler const> EulerConstPtr;

// constants requiring out of line definition

/* Commented out from here
template <typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const KDL::Euler& v)
{
  ros::message_operations::Printer< KDL::Euler >::stream(s, "", v);
  return s;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(
  const ::dh_kdl_msgs::Euler_<ContainerAllocator1>& lhs,
  const ::dh_kdl_msgs::Euler_<ContainerAllocator2>& rhs)
{
  return lhs.roll == rhs.roll && lhs.pitch == rhs.pitch && lhs.yaw == rhs.yaw;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(
  const ::dh_kdl_msgs::Euler_<ContainerAllocator1>& lhs,
  const ::dh_kdl_msgs::Euler_<ContainerAllocator2>& rhs)
{
  return !(lhs == rhs);
}
Commented out to here */
}  // namespace dh_kdl_msgs

namespace ros
{
namespace message_traits
{
// edit: removed all template parameters
template <>
struct IsMessage<KDL::Euler> : TrueType
{
};

template <>
struct IsMessage<KDL::Euler const> : TrueType
{
};

template <>
struct IsFixedSize<KDL::Euler> : TrueType
{
};

template <>
struct IsFixedSize<KDL::Euler const> : TrueType
{
};

template <>
struct HasHeader<KDL::Euler> : FalseType
{
};

template <>
struct HasHeader<KDL::Euler const> : FalseType
{
};

template <>
struct MD5Sum<KDL::Euler>
{
  static const char* value()
  {
    return "eeec8b25a660789a89540dedcb2b06d6";
  }

  static const char* value(const KDL::Euler&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xeeec8b25a660789aULL;
  static const uint64_t static_value2 = 0x89540dedcb2b06d6ULL;
};

template <>
struct DataType<KDL::Euler>
{
  static const char* value()
  {
    return "dh_kdl_msgs/Euler";
  }

  static const char* value(const KDL::Euler&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::Euler>
{
  static const char* value()
  {
    return "float64 roll   # [rad]\n"
           "float64 pitch  # [rad] \n"
           "float64 yaw    # [rad]\n";
  }

  static const char* value(const KDL::Euler&)
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
struct Serializer<KDL::Euler>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct Euler_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<KDL::Euler>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::Euler& v)
  {
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_MSGS_MESSAGE_EULER_H
