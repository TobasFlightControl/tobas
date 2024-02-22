#ifndef TOBAS_MSGS_MESSAGE_PWMARRAY_H
#define TOBAS_MSGS_MESSAGE_PWMARRAY_H

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <tobas_msgs/Pwm.h>

namespace tobas_msgs
{
template <class ContainerAllocator>
struct PwmArray_
{
  typedef PwmArray_<ContainerAllocator> Type;

  PwmArray_() : header(), pwm()
  {
  }
  PwmArray_(const ContainerAllocator& _alloc) : header(_alloc), pwm(_alloc)
  {
    (void)_alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef std::vector<
    ::tobas_msgs::Pwm_<ContainerAllocator>,
    typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<
      ::tobas_msgs::Pwm_<ContainerAllocator>>>
    _pwm_type;
  _pwm_type pwm;

  typedef boost::shared_ptr<::tobas_msgs::PwmArray_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr<::tobas_msgs::PwmArray_<ContainerAllocator> const> ConstPtr;

};  // struct PwmArray_

typedef ::tobas_msgs::PwmArray_<std::allocator<void>> PwmArray;

typedef boost::shared_ptr<::tobas_msgs::PwmArray> PwmArrayPtr;
typedef boost::shared_ptr<::tobas_msgs::PwmArray const> PwmArrayConstPtr;

// constants requiring out of line definition

template <typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tobas_msgs::PwmArray_<ContainerAllocator>& v)
{
  ros::message_operations::Printer<::tobas_msgs::PwmArray_<ContainerAllocator>>::stream(s, "", v);
  return s;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(
  const ::tobas_msgs::PwmArray_<ContainerAllocator1>& lhs,
  const ::tobas_msgs::PwmArray_<ContainerAllocator2>& rhs)
{
  return lhs.header == rhs.header && lhs.pwm == rhs.pwm;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(
  const ::tobas_msgs::PwmArray_<ContainerAllocator1>& lhs,
  const ::tobas_msgs::PwmArray_<ContainerAllocator2>& rhs)
{
  return !(lhs == rhs);
}

}  // namespace tobas_msgs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage<::tobas_msgs::PwmArray_<ContainerAllocator>> : TrueType
{
};

template <class ContainerAllocator>
struct IsMessage<::tobas_msgs::PwmArray_<ContainerAllocator> const> : TrueType
{
};

template <class ContainerAllocator>
struct IsFixedSize<::tobas_msgs::PwmArray_<ContainerAllocator>> : FalseType
{
};

template <class ContainerAllocator>
struct IsFixedSize<::tobas_msgs::PwmArray_<ContainerAllocator> const> : FalseType
{
};

template <class ContainerAllocator>
struct HasHeader<::tobas_msgs::PwmArray_<ContainerAllocator>> : TrueType
{
};

template <class ContainerAllocator>
struct HasHeader<::tobas_msgs::PwmArray_<ContainerAllocator> const> : TrueType
{
};

template <class ContainerAllocator>
struct MD5Sum<::tobas_msgs::PwmArray_<ContainerAllocator>>
{
  static const char* value()
  {
    return "c855f76b6388417710970c0953ae1933";
  }

  static const char* value(const ::tobas_msgs::PwmArray_<ContainerAllocator>&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xc855f76b63884177ULL;
  static const uint64_t static_value2 = 0x10970c0953ae1933ULL;
};

template <class ContainerAllocator>
struct DataType<::tobas_msgs::PwmArray_<ContainerAllocator>>
{
  static const char* value()
  {
    return "tobas_msgs/PwmArray";
  }

  static const char* value(const ::tobas_msgs::PwmArray_<ContainerAllocator>&)
  {
    return value();
  }
};

template <class ContainerAllocator>
struct Definition<::tobas_msgs::PwmArray_<ContainerAllocator>>
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
           "tobas_msgs/Pwm[] pwm\n"
           "\n"
           "================================================================================\n"
           "MSG: std_msgs/Header\n"
           "# Standard metadata for higher-level stamped data types.\n"
           "# This is generally used to communicate timestamped data \n"
           "# in a particular coordinate frame.\n"
           "# \n"
           "# sequence ID: consecutively increasing ID \n"
           "uint32 seq\n"
           "#Two-integer timestamp that is expressed as:\n"
           "# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called "
           "'secs')\n"
           "# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called "
           "'nsecs')\n"
           "# time-handling sugar is provided by the client library\n"
           "time stamp\n"
           "#Frame this data is associated with\n"
           "string frame_id\n"
           "\n"
           "================================================================================\n"
           "MSG: tobas_msgs/Pwm\n"
           "uint64 channel  # 0 ~ 13\n"
           "float64 period  # [us]\n";
  }

  static const char* value(const ::tobas_msgs::PwmArray_<ContainerAllocator>&)
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
struct Serializer<::tobas_msgs::PwmArray_<ContainerAllocator>>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.pwm);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct PwmArray_

}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <class ContainerAllocator>
struct Printer<::tobas_msgs::PwmArray_<ContainerAllocator>>
{
  template <typename Stream>
  static void
  stream(Stream& s, const std::string& indent, const ::tobas_msgs::PwmArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer<::std_msgs::Header_<ContainerAllocator>>::stream(s, indent + "  ", v.header);
    s << indent << "pwm[]" << std::endl;
    for (size_t i = 0; i < v.pwm.size(); ++i)
    {
      s << indent << "  pwm[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer<::tobas_msgs::Pwm_<ContainerAllocator>>::stream(s, indent + "    ", v.pwm[i]);
    }
  }
};

}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_MSGS_MESSAGE_PWMARRAY_H
