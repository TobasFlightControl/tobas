#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/rigidbodyinertia.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using RigidBodyInertia_ = kdl::RigidBodyInertia;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<kdl::RigidBodyInertia>
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
    double mass;
    kdl::Vector cog;
    kdl::RigidBodyInertia Ic;

    stream.next(mass);
    stream.next(cog);
    stream.next(Ic);

    m = T(mass, cog, Ic);
  }

  template <typename T>
  inline static uint32_t serializedLength(const T&)
  {
    return (uint32_t)(sizeof(double) + 3 * sizeof(double) + 9 * sizeof(double));
  }
};
}  // namespace serialization
}  // namespace ros
