#ifndef TOBAS_EIGEN_MSGS_MESSAGE_MATRIX3D_H
#define TOBAS_EIGEN_MSGS_MESSAGE_MATRIX3D_H

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <Eigen/Core>

namespace tobas_eigen_msgs
{
template <class ContainerAllocator>
using Matrix3d_ = Eigen::Matrix3d;

typedef tobas_eigen_msgs::Matrix3d_<std::allocator<void>> Matrix3d;

typedef boost::shared_ptr<tobas_eigen_msgs::Matrix3d> Matrix3dPtr;
typedef boost::shared_ptr<tobas_eigen_msgs::Matrix3d const> Matrix3dConstPtr;
}  // namespace tobas_eigen_msgs

namespace ros
{
namespace message_traits
{
template <>
struct IsMessage<Eigen::Matrix3d> : TrueType
{
};

template <>
struct IsMessage<Eigen::Matrix3d const> : TrueType
{
};

template <>
struct IsFixedSize<Eigen::Matrix3d> : TrueType
{
};

template <>
struct IsFixedSize<Eigen::Matrix3d const> : TrueType
{
};

template <>
struct HasHeader<Eigen::Matrix3d> : FalseType
{
};

template <>
struct HasHeader<Eigen::Matrix3d const> : FalseType
{
};

template <>
struct MD5Sum<Eigen::Matrix3d>
{
  static const char* value()
  {
    return "ca66b32e4ad9de837a30ea9fe5ade752";
  }

  static const char* value(const Eigen::Matrix3d&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xca66b32e4ad9de83ULL;
  static const uint64_t static_value2 = 0x7a30ea9fe5ade752ULL;
};

template <>
struct DataType<Eigen::Matrix3d>
{
  static const char* value()
  {
    return "tobas_eigen_msgs/Matrix3d";
  }

  static const char* value(const Eigen::Matrix3d&)
  {
    return value();
  }
};

template <>
struct Definition<Eigen::Matrix3d>
{
  static const char* value()
  {
    return "# Eigen::Matrix3d\n"
           "\n"
           "float64[9] data\n";
  }

  static const char* value(const Eigen::Matrix3d&)
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
struct Serializer<Eigen::Matrix3d>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    for (size_t row = 0; row < 3; ++row)
      for (size_t col = 0; col < 3; ++col)
        stream.next(m(row, col));
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};  // struct Matrix3d_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <>
struct Printer<Eigen::Matrix3d>
{
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const Eigen::Matrix3d& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
  }
};
}  // namespace message_operations
}  // namespace ros

#endif  // TOBAS_EIGEN_MSGS_MESSAGE_MATRIX3D_H
