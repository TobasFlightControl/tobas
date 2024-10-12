#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/Vector.hpp>

#include <tobas_hal_msgs/msg/imu.hpp>

namespace tobas_hal_msgs
{
struct Imu
{
  std_msgs::msg::Header header;
  kdl::Vector gyro;
  kdl::Vector accel;

  using SharedPtr = std::shared_ptr<Imu>;
  using ConstSharedPtr = std::shared_ptr<const Imu>;
};
}  // namespace tobas_hal_msgs

template <>
struct rclcpp::TypeAdapter<tobas_hal_msgs::Imu, tobas_hal_msgs::msg::Imu>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_hal_msgs::Imu;
  using ros_message_type = tobas_hal_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.gyro, dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.accel, dst.accel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.gyro, dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.accel, dst.accel);
  }
};

namespace tobas_hal_msgs
{
using ImuAdapter = rclcpp::TypeAdapter<tobas_hal_msgs::Imu, tobas_hal_msgs::msg::Imu>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_hal_msgs::Imu, tobas_hal_msgs::msg::Imu);
