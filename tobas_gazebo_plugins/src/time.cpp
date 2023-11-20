#include "../include/tobas_gazebo_plugins/time.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

namespace gazebo
{
ros::Duration operator-(const common::Time& lhs, const ros::Time& rhs)
{
  ros::Time lhs_ros;
  timeGazeboToRos(lhs, lhs_ros);
  return lhs_ros - rhs;
}

ros::Duration operator-(const ros::Time& lhs, const common::Time& rhs)
{
  ros::Time rhs_ros;
  timeGazeboToRos(rhs, rhs_ros);
  return lhs - rhs_ros;
}
}  // namespace gazebo
