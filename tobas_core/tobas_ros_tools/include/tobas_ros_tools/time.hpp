#pragma once

#include <chrono>
#include <ros/ros.h>

namespace tobas_ros
{
inline std::chrono::system_clock::time_point chronoFromRosTime(const ros::Time& ros_time)
{
  return std::chrono::system_clock::time_point(
    std::chrono::seconds(ros_time.sec) + std::chrono::nanoseconds(ros_time.nsec));
}
}  // namespace tobas_ros
