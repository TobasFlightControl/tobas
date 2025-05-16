#include "tobas_gazebo_system_plugins/conversions/gazebo_msg.hpp"

namespace gazebo
{
void vectorGazeboToMsg(const gz::math::Vector3d& g, geometry_msgs::msg::Vector3& m)
{
  m.x = g.X();
  m.y = g.Y();
  m.z = g.Z();
}

void vectorMsgToGazebo(const geometry_msgs::msg::Vector3& m, gz::math::Vector3d& g)
{
  g.Set(m.x, m.y, m.z);
}

void pointGazeboToMsg(const gz::math::Vector3d& g, geometry_msgs::msg::Point& m)
{
  m.x = g.X();
  m.y = g.Y();
  m.z = g.Z();
}

void pointMsgToGazebo(const geometry_msgs::msg::Point& m, gz::math::Vector3d& g)
{
  g.Set(m.x, m.y, m.z);
}

void quaternionGazeboToMsg(const gz::math::Quaterniond& g, geometry_msgs::msg::Quaternion& m)
{
  m.w = g.W();
  m.x = g.X();
  m.y = g.Y();
  m.z = g.Z();
}

void quaternionMsgToGazebo(const geometry_msgs::msg::Quaternion& m, gz::math::Quaterniond& g)
{
  g.Set(m.w, m.x, m.y, m.z);
}

void poseGazeboToMsg(const gz::math::Pose3d& g, geometry_msgs::msg::Pose& m)
{
  pointGazeboToMsg(g.Pos(), m.position);
  quaternionGazeboToMsg(g.Rot(), m.orientation);
}

void poseMsgToGazebo(const geometry_msgs::msg::Pose& m, gz::math::Pose3d& g)
{
  pointMsgToGazebo(m.position, g.Pos());
  quaternionMsgToGazebo(m.orientation, g.Rot());
}
}  // namespace gazebo
