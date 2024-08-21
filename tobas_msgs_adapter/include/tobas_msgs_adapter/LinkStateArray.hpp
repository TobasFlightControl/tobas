#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_msgs/msg/link_state_array.hpp>

#include "./LinkState.hpp"

namespace tobas_msgs
{
struct LinkStateArray
{
  std_msgs::msg::Header header;
  std::vector<tobas_msgs::LinkState> states;

  using SharedPtr = std::shared_ptr<LinkStateArray>;
  using ConstSharedPtr = std::shared_ptr<const LinkStateArray>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::LinkStateArray, tobas_msgs::msg::LinkStateArray>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::LinkStateArray;
  using ros_message_type = tobas_msgs::msg::LinkStateArray;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;

    dst.states.clear();
    for (const auto& state : src.states)
    {
      dst.states.emplace_back();
      tobas_msgs::LinkStateAdapter::convert_to_ros_message(state, dst.states.back());
    }
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;

    dst.states.clear();
    for (const auto& state : src.states)
    {
      dst.states.emplace_back();
      tobas_msgs::LinkStateAdapter::convert_to_custom(state, dst.states.back());
    }
  }
};

namespace tobas_msgs
{
using LinkStateArrayAdapter = rclcpp::TypeAdapter<tobas_msgs::LinkStateArray, tobas_msgs::msg::LinkStateArray>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::LinkStateArray, tobas_msgs::msg::LinkStateArray);
