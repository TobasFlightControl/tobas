#ifndef TOBAS_MSGS_MESSAGE_PWM_H
#define TOBAS_MSGS_MESSAGE_PWM_H

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

namespace tobas_msgs
{
template <class ContainerAllocator>
struct Pwm_
{
  typedef Pwm_<ContainerAllocator> Type;

  Pwm_() : channel(0), period(0.0)
  {
  }
  Pwm_(const ContainerAllocator& _alloc) : channel(0), period(0.0)
  {
    (void)_alloc;
  }

  /* ADDED */
  Pwm_(const size_t& _channel, const double& _period) : channel(_channel), period(_period)
  {
  }

  typedef uint64_t _channel_type;
  _channel_type channel;

  typedef double _period_type;
  _period_type period;

  typedef boost::shared_ptr< ::tobas_msgs::Pwm_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tobas_msgs::Pwm_<ContainerAllocator> const> ConstPtr;

};  // struct Pwm_

typedef ::tobas_msgs::Pwm_<std::allocator<void> > Pwm;

typedef boost::shared_ptr< ::tobas_msgs::Pwm> PwmPtr;
typedef boost::shared_ptr< ::tobas_msgs::Pwm const> PwmConstPtr;

// constants requiring out of line definition

template <typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tobas_msgs::Pwm_<ContainerAllocator>& v)
{
  ros::message_operations::Printer< ::tobas_msgs::Pwm_<ContainerAllocator> >::stream(s, "", v);
  return s;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(
  const ::tobas_msgs::Pwm_<ContainerAllocator1>& lhs,
  const ::tobas_msgs::Pwm_<ContainerAllocator2>& rhs)
{
  return lhs.channel == rhs.channel && lhs.period == rhs.period;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(
  const ::tobas_msgs::Pwm_<ContainerAllocator1>& lhs,
  const ::tobas_msgs::Pwm_<ContainerAllocator2>& rhs)
{
  return !(lhs == rhs);
}

}  // namespace tobas_msgs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::tobas_msgs::Pwm_<ContainerAllocator> > : TrueType
{
};

template <class ContainerAllocator>
struct IsMessage< ::tobas_msgs::Pwm_<ContainerAllocator> const> : TrueType
{
};

template <class ContainerAllocator>
struct IsFixedSize< ::tobas_msgs::Pwm_<ContainerAllocator> > : TrueType
{
};

template <class ContainerAllocator>
struct IsFixedSize< ::tobas_msgs::Pwm_<ContainerAllocator> const> : TrueType
{
};

template <class ContainerAllocator>
struct HasHeader< ::tobas_msgs::Pwm_<ContainerAllocator> > : FalseType
{
};

template <class ContainerAllocator>
struct HasHeader< ::tobas_msgs::Pwm_<ContainerAllocator> const> : FalseType
{
};

template <class ContainerAllocator>
struct MD5Sum< ::tobas_msgs::Pwm_<ContainerAllocator> >
{
  static const char* value()
  {
    return "10be13892bd1a60fa0d6487dee97281c";
  }

  static const char* value(const ::tobas_msgs::Pwm_<ContainerAllocator>&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x10be13892bd1a60fULL;
  static const uint64_t static_value2 = 0xa0d6487dee97281cULL;
};

template <class ContainerAllocator>
struct DataType< ::tobas_msgs::Pwm_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tobas_msgs/Pwm";
  }

  static const char* value(const ::tobas_msgs::Pwm_<ContainerAllocator>&)
  {
    return value();
  }
};

template <class ContainerAllocator>
struct Definition< ::tobas_msgs::Pwm_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 channel  # 0 ~ 13\n"
           "float64 period  # [us]\n";
  }

  static const char* value(const ::tobas_msgs::Pwm_<ContainerAllocator>&)
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
template <class ContainerAllocator>
struct Serializer< ::tobas_msgs::Pwm_<ContainerAllocator> >
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.channel);
    stream.next(m.period);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct Pwm_

}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <class ContainerAllocator>
struct Printer< ::tobas_msgs::Pwm_<ContainerAllocator> >
{
  template <typename Stream>
  static void
  stream(Stream& s, const std::string& indent, const ::tobas_msgs::Pwm_<ContainerAllocator>& v)
  {
    s << indent << "channel: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.channel);
    s << indent << "period: ";
    Printer<double>::stream(s, indent + "  ", v.period);
  }
};

}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_MSGS_MESSAGE_PWM_H
