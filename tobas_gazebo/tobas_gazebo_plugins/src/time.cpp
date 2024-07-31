#include "../include/tobas_gazebo_plugins/time.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

namespace gazebo
{
rclcpp::Duration operator-(const common::Time& lhs, const rclcpp::Time& rhs)
{
  rclcpp::Time lhs_ros;
  timeGazeboToRos(lhs, lhs_ros);
  return lhs_ros - rhs;
}

rclcpp::Duration operator-(const rclcpp::Time& lhs, const common::Time& rhs)
{
  rclcpp::Time rhs_ros;
  timeGazeboToRos(rhs, rhs_ros);
  return lhs - rhs_ros;
}
}  // namespace gazebo
