#pragma once

#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tobas
{
using StringCallback = std::function<void(const std::string&)>;

/**
 * @brief SynchronizedStringParameter is a way to load a string from the ROS environment.
 *
 * First it tries to load the string from a parameter.
 * If that fails, it subscribes to a std_msgs::String topic of the same name to get the value.
 *
 * If the parameter is loaded successfully, you can publish the value as a String msg if the publish_NAME param is true.
 *
 * You can specify how long to wait for a subscribed message with NAME_timeout (double in seconds)
 *
 * By default, the subscription will be killed after the first message is received.
 * If the parameter NAME_continuous is true, then the parent_callback will be called on every subsequent message.
 */
class SynchronizedStringParameter
{
public:
  std::string loadInitialValue(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& name,
    const StringCallback& parent_callback = {},
    bool default_continuous_value = false,
    double default_timeout = 10.0);

protected:
  bool getMainParameter();

  bool shouldPublish();

  bool waitForMessage(const rclcpp::Duration& timeout);

  void stringCallback(const std_msgs::msg::String::ConstSharedPtr& msg);

  std::shared_ptr<rclcpp::Node> node_;
  std::string name_;
  StringCallback parent_callback_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;

  std::string content_;
};
}  // namespace tobas
