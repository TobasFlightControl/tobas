#pragma once

#include <ros/serialization.h>

#include <tobas_kdl/jntarray.hpp>

#include "./util/serialization.hpp"

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using JntArray_ = kdl::JntArray;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace serialization
{
template <>
struct Serializer<kdl::JntArray>
{
  template <typename Stream, typename T>
  inline static void write(Stream& stream, const T& m)
  {
    serialize(stream, m.data.data(), m.data.rows());
  }

  template <typename Stream, typename T>
  inline static void read(Stream& stream, T& m)
  {
    IStream peek_size_stream(stream.getData(), stream.getLength());
    uint32_t size;
    peek_size_stream.next(size);
    m.resize(size);
    deserialize(stream, m.data.data(), m.data.rows());
  }

  template <typename T>
  inline static uint32_t serializedLength(const T& m)
  {
    return serializationLength(m.data.data(), m.data.rows());
  }
};
}  // namespace serialization
}  // namespace ros
