#pragma once

#include <ros/serialization.h>

namespace tobas_msgs
{
template <class ContainerAllocator>
struct Pwm_
{
  typedef Pwm_<ContainerAllocator> Type;

  Pwm_() : channel(0), period(0)
  {
  }

  Pwm_(const ContainerAllocator& _alloc) : channel(0), period(0)
  {
    (void)_alloc;
  }

  /* ADDED */
  Pwm_(const uint8_t& _channel, const uint16_t& _period) : channel(_channel), period(_period)
  {
  }

  typedef uint8_t _channel_type;
  _channel_type channel;

  typedef uint16_t _period_type;
  _period_type period;
};

typedef ::tobas_msgs::Pwm_<std::allocator<void>> Pwm;
}  // namespace tobas_msgs

namespace ros
{
namespace serialization
{
template <class ContainerAllocator>
struct Serializer<::tobas_msgs::Pwm_<ContainerAllocator>>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.channel);
    stream.next(m.period);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}  // namespace serialization
}  // namespace ros
