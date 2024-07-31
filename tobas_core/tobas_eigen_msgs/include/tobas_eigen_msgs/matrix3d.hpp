#pragma once

#include <rclcpp/type_adapter.hpp>
#include <eigen3/Eigen/Core>

#include <tobas_eigen_msgs/msg/matrix3d.hpp>

template <>
struct rclcpp::TypeAdapter<Eigen::Matrix3d, tobas_eigen_msgs::msg::Matrix3d>
{
  using is_specialized = std::true_type;
  using custom_type = Eigen::Matrix3d;
  using ros_message_type = tobas_eigen_msgs::msg::Matrix3d;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    for (size_t r = 0; r < 3; ++r)
      for (size_t c = 0; c < 3; ++c)
        dst.data[r * 3 + c] = src(r, c);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    for (size_t r = 0; r < 3; ++r)
      for (size_t c = 0; c < 3; ++c)
        dst(r, c) = src.data[r * 3 + c];
  }
};
