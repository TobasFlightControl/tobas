#pragma once

#include <ros/serialization.h>

namespace tobas_msgs
{
template <class ContainerAllocator>
struct Throttle_
{
  typedef Throttle_<ContainerAllocator> Type;

  Throttle_() : channel(0), throttle(0.0)
  {
  }

  Throttle_(const ContainerAllocator& _alloc) : channel(0), throttle(0.0)
  {
    (void)_alloc;
  }

  /* ADDED */
  Throttle_(const uint8_t& _channel, const double& _throttle) : channel(_channel), throttle(_throttle)
  {
  }

  typedef uint8_t _channel_type;
  _channel_type channel;

  typedef double _throttle_type;
  _throttle_type throttle;
};

typedef ::tobas_msgs::Throttle_<std::allocator<void>> Throttle;
}  // namespace tobas_msgs

namespace ros
{
namespace serialization
{
template <class ContainerAllocator>
struct Serializer<::tobas_msgs::Throttle_<ContainerAllocator>>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.channel);
    stream.next(m.throttle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
