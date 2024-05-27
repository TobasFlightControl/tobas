#pragma once

#include <tf2_ros/transform_listener.h>

namespace tobas_ros
{
class TransformListener
{
public:
  explicit TransformListener();

  bool lookupTransform(const std::string& parent, const std::string& child, const ros::Time& time = ros::Time(0));

  inline const geometry_msgs::TransformStamped& getTransform();
  inline const char* getErrorMessage();

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::TransformStamped tf_;
  const char* error_msg_;
};

inline const geometry_msgs::TransformStamped& TransformListener::getTransform()
{
  return tf_;
}

inline const char* TransformListener::getErrorMessage()
{
  return error_msg_;
}
}  // namespace tobas_ros
