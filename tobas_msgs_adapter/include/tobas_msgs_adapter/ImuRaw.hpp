#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/Vector.hpp>

#include <tobas_msgs/msg/imu_raw.hpp>

namespace tobas_msgs
{
struct ImuRaw
{
  std_msgs::msg::Header header;
  kdl::Vector gyro;
  kdl::Vector accel;

  using SharedPtr = std::shared_ptr<ImuRaw>;
  using ConstSharedPtr = std::shared_ptr<const ImuRaw>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::ImuRaw, tobas_msgs::msg::ImuRaw>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::ImuRaw;
  using ros_message_type = tobas_msgs::msg::ImuRaw;

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

namespace tobas_msgs
{
using ImuRawAdapter = rclcpp::TypeAdapter<tobas_msgs::ImuRaw, tobas_msgs::msg::ImuRaw>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::ImuRaw, tobas_msgs::msg::ImuRaw);
