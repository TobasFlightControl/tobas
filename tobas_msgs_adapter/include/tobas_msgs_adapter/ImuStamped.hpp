#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_msgs/msg/imu_stamped.hpp>

#include "./Imu.hpp"

namespace tobas_msgs
{
struct ImuStamped
{
  std_msgs::msg::Header header;
  Imu imu;

  using SharedPtr = std::shared_ptr<ImuStamped>;
  using ConstSharedPtr = std::shared_ptr<const ImuStamped>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::ImuStamped, tobas_msgs::msg::ImuStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::ImuStamped;
  using ros_message_type = tobas_msgs::msg::ImuStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::ImuAdapter::convert_to_ros_message(src.imu, dst.imu);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::ImuAdapter::convert_to_custom(src.imu, dst.imu);
  }
};

namespace tobas_msgs
{
using ImuStampedAdapter = rclcpp::TypeAdapter<tobas_msgs::ImuStamped, tobas_msgs::msg::ImuStamped>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::ImuStamped, tobas_msgs::msg::ImuStamped);
