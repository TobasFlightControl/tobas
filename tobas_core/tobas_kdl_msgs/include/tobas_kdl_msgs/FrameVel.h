#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/framevel.hpp>

#include "./Frame.h"
#include "./Twist.h"

namespace tobas_kdl_msgs
{
template <class ContainerAllocator>
using FrameVel_ = tobas_kdl::FrameVel;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<tobas_kdl::FrameVel>
{
  template <typename Stream, typename T>
  inline static void write(Stream& stream, const T& m)
  {
    stream.next(m.getFrame());
    stream.next(m.getTwist());
  }

  template <typename Stream, typename T>
  inline static void read(Stream& stream, T& m)
  {
    tobas_kdl::Frame frame;
    tobas_kdl::Twist twist;

    stream.next(frame);
    stream.next(twist);

    m = T(frame, twist);
  }

  template <typename T>
  inline static uint32_t serializedLength(const T&)
  {
    return (uint32_t)(12 * sizeof(double) + 6 * sizeof(double));
  }
};
}  // namespace serialization
}  // namespace ros
