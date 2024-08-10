#include "../../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace gz::math;

namespace gazebo
{
void vectorGazeboToRos(const Vector3d& g, geometry_msgs::msg::Vector3& r)
{
  r.x = g.X();
  r.y = g.Y();
  r.z = g.Z();
}

void vectorRosToGazebo(const geometry_msgs::msg::Vector3& r, Vector3d& g)
{
  g.X() = r.x;
  g.Y() = r.y;
  g.Z() = r.z;
}

void pointGazeboToRos(const Vector3d& g, geometry_msgs::msg::Point& r)
{
  r.x = g.X();
  r.y = g.Y();
  r.z = g.Z();
}

void pointRosToGazebo(const geometry_msgs::msg::Point& r, Vector3d& g)
{
  g.X() = r.x;
  g.Y() = r.y;
  g.Z() = r.z;
}

void quaternionGazeboToRos(const Quaterniond& g, geometry_msgs::msg::Quaternion& r)
{
  r.w = g.W();
  r.x = g.X();
  r.y = g.Y();
  r.z = g.Z();
}

void quaternionRosToGazebo(const geometry_msgs::msg::Quaternion& r, Quaterniond& g)
{
  g.W() = r.w;
  g.X() = r.x;
  g.Y() = r.y;
  g.Z() = r.z;
}

void poseGazeboToRos(const Pose3d& g, geometry_msgs::msg::Pose& r)
{
  pointGazeboToRos(g.Pos(), r.position);
  quaternionGazeboToRos(g.Rot(), r.orientation);
}

void poseRosToGazebo(const geometry_msgs::msg::Pose& r, Pose3d& g)
{
  pointRosToGazebo(r.position, g.Pos());
  quaternionRosToGazebo(r.orientation, g.Rot());
}
}  // namespace gazebo
