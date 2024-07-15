#pragma once

#include <ros/serialization.h>

#include <tobas_kdl_msgs/Frame.h>
#include <tobas_kdl_msgs/Twist.h>
#include <tobas_kdl_msgs/Accel.h>
#include <tobas_kdl_msgs/Wrench.h>

namespace tobas_msgs
{
template <class ContainerAllocator>
struct LinkState_
{
  typedef LinkState_<ContainerAllocator> Type;

  LinkState_() : name(), frame(), twist(), accel(), wrench()
  {
  }

  LinkState_(const ContainerAllocator& _alloc)
    : name(_alloc), frame(_alloc), twist(_alloc), accel(_alloc), wrench(_alloc)
  {
    (void)_alloc;
  }

  /* ADDED */
  LinkState_(
    const std::string& _name,
    const kdl::Frame& _frame,
    const kdl::Twist& _twist,
    const kdl::Accel& _accel,
    const kdl::Wrench& _wrench)
    : name(_name), frame(_frame), twist(_twist), accel(_accel), wrench(_wrench)
  {
  }

  typedef std::basic_string<
    char,
    std::char_traits<char>,
    typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
    _name_type;
  _name_type name;

  typedef tobas_kdl_msgs::Frame_<ContainerAllocator> _frame_type;
  _frame_type frame;

  typedef tobas_kdl_msgs::Twist_<ContainerAllocator> _twist_type;
  _twist_type twist;

  typedef tobas_kdl_msgs::Accel_<ContainerAllocator> _accel_type;
  _accel_type accel;

  typedef tobas_kdl_msgs::Wrench_<ContainerAllocator> _wrench_type;
  _wrench_type wrench;
};

typedef ::tobas_msgs::LinkState_<std::allocator<void>> LinkState;
}  // namespace tobas_msgs

namespace ros
{
namespace serialization
{
template <class ContainerAllocator>
struct Serializer<::tobas_msgs::LinkState_<ContainerAllocator>>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.frame);
    stream.next(m.twist);
    stream.next(m.accel);
    stream.next(m.wrench);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
