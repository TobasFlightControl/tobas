#pragma once

#include <std_msgs/ColorRGBA.h>

namespace moveit_rviz_plugin
{
inline bool operator==(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
{
  return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
}

inline bool operator!=(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
{
  return a.r != b.r || a.g != b.g || a.b != b.b || a.a != b.a;
}
}  // namespace moveit_rviz_plugin
