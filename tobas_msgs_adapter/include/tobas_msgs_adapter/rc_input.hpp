#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_constants/flight_mode.hpp>

#include <tobas_msgs/msg/rc_input.hpp>

namespace tobas_msgs
{
struct RCInput
{
  std_msgs::msg::Header header;
  double roll;
  double pitch;
  double throttle;
  double yaw;
  bool enable;
  tobas::flight_mode_t mode;
  bool gpsw;

  using SharedPtr = std::shared_ptr<RCInput>;
  using ConstSharedPtr = std::shared_ptr<const RCInput>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::RCInput, tobas_msgs::msg::RCInput>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::RCInput;
  using ros_message_type = tobas_msgs::msg::RCInput;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.roll = src.roll;
    dst.pitch = src.pitch;
    dst.throttle = src.throttle;
    dst.yaw = src.yaw;
    dst.enable = src.enable;
    dst.mode = static_cast<uint8_t>(src.mode);
    dst.gpsw = src.gpsw;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.roll = src.roll;
    dst.pitch = src.pitch;
    dst.throttle = src.throttle;
    dst.yaw = src.yaw;
    dst.enable = src.enable;
    dst.mode = static_cast<tobas::flight_mode_t>(src.mode);
    dst.gpsw = src.gpsw;
  }
};

namespace tobas_msgs
{
using RCInputAdapter = rclcpp::TypeAdapter<tobas_msgs::RCInput, tobas_msgs::msg::RCInput>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::RCInput, tobas_msgs::msg::RCInput);
