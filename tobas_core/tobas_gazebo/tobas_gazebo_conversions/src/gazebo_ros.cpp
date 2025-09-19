#include "tobas_gazebo_conversions/gazebo_ros.hpp"

namespace gazebo
{
void vectorGazeboToRos(const gz::math::Vector3d& g, geometry_msgs::msg::Vector3& m)
{
  m.x = g.X();
  m.y = g.Y();
  m.z = g.Z();
}

void vectorRosToGazebo(const geometry_msgs::msg::Vector3& m, gz::math::Vector3d& g)
{
  g.Set(m.x, m.y, m.z);
}

void pointGazeboToRos(const gz::math::Vector3d& g, geometry_msgs::msg::Point& m)
{
  m.x = g.X();
  m.y = g.Y();
  m.z = g.Z();
}

void pointRosToGazebo(const geometry_msgs::msg::Point& m, gz::math::Vector3d& g)
{
  g.Set(m.x, m.y, m.z);
}

void quaternionGazeboToRos(const gz::math::Quaterniond& g, geometry_msgs::msg::Quaternion& m)
{
  m.w = g.W();
  m.x = g.X();
  m.y = g.Y();
  m.z = g.Z();
}

void quaternionRosToGazebo(const geometry_msgs::msg::Quaternion& m, gz::math::Quaterniond& g)
{
  g.Set(m.w, m.x, m.y, m.z);
}

void poseGazeboToRos(const gz::math::Pose3d& g, geometry_msgs::msg::Pose& m)
{
  pointGazeboToRos(g.Pos(), m.position);
  quaternionGazeboToRos(g.Rot(), m.orientation);
}

void poseRosToGazebo(const geometry_msgs::msg::Pose& m, gz::math::Pose3d& g)
{
  pointRosToGazebo(m.position, g.Pos());
  quaternionRosToGazebo(m.orientation, g.Rot());
}
}  // namespace gazebo
