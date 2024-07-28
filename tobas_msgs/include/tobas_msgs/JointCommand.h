#pragma once

#include <ros/serialization.h>

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
  JointCommand_(const std::string& _name, const double& _data) : name(_name), data(_data)
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
};

typedef ::tobas_msgs::JointCommand_<std::allocator<void>> JointCommand;
}  // namespace tobas_msgs

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
};
}  // namespace serialization
}  // namespace ros
