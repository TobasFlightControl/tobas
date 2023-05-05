#include "../../../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

namespace gazebo
{
void timeGazeboToRos(const common::Time& g, ros::Time& r)
{
  r.sec = g.sec;
  r.nsec = g.nsec;
}

void timeRosToGazebo(const ros::Time& r, common::Time& g)
{
  g.sec = r.sec;
  g.nsec = r.nsec;
}
}  // namespace gazebo
