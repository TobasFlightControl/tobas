#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_msgs/msg/imu.hpp>

#include "./imu.hpp"

namespace tobas_msgs
{
struct Imu
{
  std_msgs::msg::Header header;
  kdl::Vector accel;
  kdl::Vector gyro;
  kdl::Vector dgyro;

  using SharedPtr = std::shared_ptr<Imu>;
  using ConstSharedPtr = std::shared_ptr<const Imu>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::Imu, tobas_msgs::msg::Imu>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::Imu;
  using ros_message_type = tobas_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.accel, dst.accel);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.gyro, dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.dgyro, dst.dgyro);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.accel, dst.accel);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.gyro, dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.dgyro, dst.dgyro);
  }
};

namespace tobas_msgs
{
using ImuAdapter = rclcpp::TypeAdapter<tobas_msgs::Imu, tobas_msgs::msg::Imu>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::Imu, tobas_msgs::msg::Imu);
