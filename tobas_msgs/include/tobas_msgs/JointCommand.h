#ifndef TOBAS_MSGS_MESSAGE_JOINTCOMMAND_H
#define TOBAS_MSGS_MESSAGE_JOINTCOMMAND_H

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
struct JointCommand_
{
  typedef JointCommand_<ContainerAllocator> Type;

  JointCommand_() : name(), data(0.0)
  {
  }
  JointCommand_(const ContainerAllocator& _alloc) : name(_alloc), data(0.0)
  {
    (void)_alloc;
  }

  /* ADDED */
  JointCommand_(const std::string& _name, double _data) : name(_name), data(_data)
  {
  }

  typedef std::basic_string<
    char,
    std::char_traits<char>,
    typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
    _name_type;
  _name_type name;

  typedef double _data_type;
  _data_type data;

  typedef boost::shared_ptr<::tobas_msgs::JointCommand_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr<::tobas_msgs::JointCommand_<ContainerAllocator> const> ConstPtr;

};  // struct JointCommand_

typedef ::tobas_msgs::JointCommand_<std::allocator<void>> JointCommand;

typedef boost::shared_ptr<::tobas_msgs::JointCommand> JointCommandPtr;
typedef boost::shared_ptr<::tobas_msgs::JointCommand const> JointCommandConstPtr;

// constants requiring out of line definition

template <typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tobas_msgs::JointCommand_<ContainerAllocator>& v)
{
  ros::message_operations::Printer<::tobas_msgs::JointCommand_<ContainerAllocator>>::stream(
    s, "", v);
  return s;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(
  const ::tobas_msgs::JointCommand_<ContainerAllocator1>& lhs,
  const ::tobas_msgs::JointCommand_<ContainerAllocator2>& rhs)
{
  return lhs.name == rhs.name && lhs.data == rhs.data;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(
  const ::tobas_msgs::JointCommand_<ContainerAllocator1>& lhs,
  const ::tobas_msgs::JointCommand_<ContainerAllocator2>& rhs)
{
  return !(lhs == rhs);
}

}  // namespace tobas_msgs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage<::tobas_msgs::JointCommand_<ContainerAllocator>> : TrueType
{
};

template <class ContainerAllocator>
struct IsMessage<::tobas_msgs::JointCommand_<ContainerAllocator> const> : TrueType
{
};

template <class ContainerAllocator>
struct IsFixedSize<::tobas_msgs::JointCommand_<ContainerAllocator>> : FalseType
{
};

template <class ContainerAllocator>
struct IsFixedSize<::tobas_msgs::JointCommand_<ContainerAllocator> const> : FalseType
{
};

template <class ContainerAllocator>
struct HasHeader<::tobas_msgs::JointCommand_<ContainerAllocator>> : FalseType
{
};

template <class ContainerAllocator>
struct HasHeader<::tobas_msgs::JointCommand_<ContainerAllocator> const> : FalseType
{
};

template <class ContainerAllocator>
struct MD5Sum<::tobas_msgs::JointCommand_<ContainerAllocator>>
{
  static const char* value()
  {
    return "c62b160c39494f9cfcbbd7a0e565a3eb";
  }

  static const char* value(const ::tobas_msgs::JointCommand_<ContainerAllocator>&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xc62b160c39494f9cULL;
  static const uint64_t static_value2 = 0xfcbbd7a0e565a3ebULL;
};

template <class ContainerAllocator>
struct DataType<::tobas_msgs::JointCommand_<ContainerAllocator>>
{
  static const char* value()
  {
    return "tobas_msgs/JointCommand";
  }

  static const char* value(const ::tobas_msgs::JointCommand_<ContainerAllocator>&)
  {
    return value();
  }
};

template <class ContainerAllocator>
struct Definition<::tobas_msgs::JointCommand_<ContainerAllocator>>
{
  static const char* value()
  {
    return "string name\n"
           "float64 data\n";
  }

  static const char* value(const ::tobas_msgs::JointCommand_<ContainerAllocator>&)
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
struct Serializer<::tobas_msgs::JointCommand_<ContainerAllocator>>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct JointCommand_

}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <class ContainerAllocator>
struct Printer<::tobas_msgs::JointCommand_<ContainerAllocator>>
{
  template <typename Stream>
  static void stream(
    Stream& s,
    const std::string& indent,
    const ::tobas_msgs::JointCommand_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<
      char, std::char_traits<char>,
      typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::
      stream(s, indent + "  ", v.name);
    s << indent << "data: ";
    Printer<double>::stream(s, indent + "  ", v.data);
  }
};

}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_MSGS_MESSAGE_JOINTCOMMAND_H
