#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>

namespace gazebo
{
ros::Duration operator-(const common::Time& lhs, const ros::Time& rhs);

ros::Duration operator-(const ros::Time& lhs, const common::Time& rhs);
}  // namespace gazebo
