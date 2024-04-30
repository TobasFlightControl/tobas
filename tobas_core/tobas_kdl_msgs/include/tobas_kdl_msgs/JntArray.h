#ifndef TOBAS_KDL_MSGS_MESSAGE_JNTARRAY_H
#define TOBAS_KDL_MSGS_MESSAGE_JNTARRAY_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tobas_kdl/jntarray.hpp>

namespace tobas_kdl_msgs
{
template <typename ContainerAllocator>
using JntArray_ = KDL::JntArray;

typedef ::tobas_kdl_msgs::JntArray_<std::allocator<void> > JntArray;

typedef boost::shared_ptr< ::tobas_kdl_msgs::JntArray> JntArrayPtr;
typedef boost::shared_ptr< ::tobas_kdl_msgs::JntArray const> JntArrayConstPtr;
}  // namespace tobas_kdl_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsFixedSize<KDL::JntArray> : FalseType
{
};

template <>
struct IsFixedSize<KDL::JntArray const> : FalseType
{
};

template <>
struct IsMessage<KDL::JntArray> : TrueType
{
};

template <>
struct IsMessage<KDL::JntArray const> : TrueType
{
};

template <>
struct HasHeader<KDL::JntArray> : FalseType
{
};

template <>
struct HasHeader<KDL::JntArray const> : FalseType
{
};

template <>
struct MD5Sum<KDL::JntArray>
{
  static const char* value()
  {
    return "788898178a3da2c3718461eecda8f714";
  }

  static const char* value(const KDL::JntArray&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x788898178a3da2c3ULL;
  static const uint64_t static_value2 = 0x718461eecda8f714ULL;
};

template <>
struct DataType<KDL::JntArray>
{
  static const char* value()
  {
    return "tobas_kdl_msgs/JntArray";
  }

  static const char* value(const KDL::JntArray&)
  {
    return value();
  }
};

template <>
struct Definition<KDL::JntArray>
{
  static const char* value()
  {
    return "float64[] data\n";
  }

  static const char* value(const KDL::JntArray&)
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
template <>
struct Serializer<KDL::JntArray>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct JntArray_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<KDL::JntArray>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const KDL::JntArray& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.rows(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v(i));
    }
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_KDL_MSGS_MESSAGE_JNTARRAY_H
