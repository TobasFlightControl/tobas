#pragma once

#include <eigen3/Eigen/Core>
#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs/msg/matrix3d.hpp>
#include <tobas_eigen_msgs/msg/matrix4d.hpp>
#include <tobas_eigen_msgs/msg/matrix6d.hpp>
#include <tobas_eigen_msgs/msg/vector3d.hpp>
#include <tobas_eigen_msgs/msg/vector4d.hpp>
#include <tobas_eigen_msgs/msg/vector6d.hpp>

#define DEFINE_EIGEN_MATRIX_ADAPTER(MsgType, AdapterType, Scalar, Rows, Cols)                                          \
  template <>                                                                                                          \
  struct rclcpp::TypeAdapter<Eigen::Matrix<Scalar, Rows, Cols>, MsgType>                                               \
  {                                                                                                                    \
    using is_specialized = std::true_type;                                                                             \
    using custom_type = Eigen::Matrix<Scalar, Rows, Cols>;                                                             \
    using ros_message_type = MsgType;                                                                                  \
                                                                                                                       \
    static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)                                  \
    {                                                                                                                  \
      for (int r = 0; r < Rows; ++r)                                                                                   \
        for (int c = 0; c < Cols; ++c)                                                                                 \
          dst.data[r * Cols + c] = src(r, c);                                                                          \
    }                                                                                                                  \
                                                                                                                       \
    static void convert_to_custom(const ros_message_type& src, custom_type& dst)                                       \
    {                                                                                                                  \
      for (int r = 0; r < Rows; ++r)                                                                                   \
        for (int c = 0; c < Cols; ++c)                                                                                 \
          dst(r, c) = src.data[r * Cols + c];                                                                          \
    }                                                                                                                  \
  };                                                                                                                   \
  namespace tobas_eigen_msgs                                                                                           \
  {                                                                                                                    \
  using AdapterType = rclcpp::TypeAdapter<Eigen::Matrix<Scalar, Rows, Cols>, MsgType>;                                 \
  }

#define DEFINE_EIGEN_VECTOR_ADAPTER(MsgType, AdapterType, Scalar, Size)                                                \
  DEFINE_EIGEN_MATRIX_ADAPTER(MsgType, AdapterType, Scalar, Size, 1)

DEFINE_EIGEN_MATRIX_ADAPTER(tobas_eigen_msgs::msg::Matrix3d, Matrix3dAdapter, double, 3, 3);
DEFINE_EIGEN_MATRIX_ADAPTER(tobas_eigen_msgs::msg::Matrix4d, Matrix4dAdapter, double, 4, 4);
DEFINE_EIGEN_MATRIX_ADAPTER(tobas_eigen_msgs::msg::Matrix6d, Matrix6dAdapter, double, 6, 6);
DEFINE_EIGEN_VECTOR_ADAPTER(tobas_eigen_msgs::msg::Vector3d, Vector3dAdapter, double, 3);
DEFINE_EIGEN_VECTOR_ADAPTER(tobas_eigen_msgs::msg::Vector4d, Vector4dAdapter, double, 4);
DEFINE_EIGEN_VECTOR_ADAPTER(tobas_eigen_msgs::msg::Vector6d, Vector6dAdapter, double, 6);
