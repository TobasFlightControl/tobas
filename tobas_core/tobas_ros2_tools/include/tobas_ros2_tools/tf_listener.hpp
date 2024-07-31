#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ros2
{
class TransformListener
{
public:
  explicit TransformListener(rclcpp::Node::SharedPtr node);

  bool lookupTransform(const std::string& parent, const std::string& child, const rclcpp::Time& time = rclcpp::Time(0));

  inline const geometry_msgs::msg::TransformStamped& getTransform();
  inline const char* getErrorMessage();

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::TransformStamped tf_;
  const char* error_msg_;
};

inline const geometry_msgs::msg::TransformStamped& TransformListener::getTransform()
{
  return tf_;
}

inline const char* TransformListener::getErrorMessage()
{
  return error_msg_;
}
}  // namespace ros2
