#include <dh_kdl/conversion/kdl_msg.hpp>

#include "../include/tobas_msgs/conversions/msg_msg.hpp"

using namespace KDL;

namespace tobas
{
void poseTobasToMsg(const tobas_msgs::Pose& t, geometry_msgs::Pose& m)
{
  pointKDLToMsg(t.pos, m.position);

  const auto rot = Rotation::RPY(t.euler.roll, t.euler.pitch, t.euler.yaw);
  rot.GetQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
}

void odometryTobasToMsg(const tobas_msgs::PoseTwist& t, nav_msgs::Odometry& m)
{
  m.header = t.header;

  poseTobasToMsg(t.pose, m.pose.pose);
  twistKDLToMsg(t.twist, m.twist.twist);

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      m.pose.covariance[6 * i + j] = t.position_covariance[3 * i + j];
      m.pose.covariance[6 * (i + 3) + (j + 3)] = t.orientation_covariance[3 * i + j];
      m.pose.covariance[6 * i + (j + 3)] = 0.;
      m.pose.covariance[6 * (i + 3) + j] = 0.;

      m.twist.covariance[6 * i + j] = t.linear_velocity_covariance[3 * i + j];
      m.twist.covariance[6 * (i + 3) + (j + 3)] = t.angular_velocity_covariance[3 * i + j];
      m.twist.covariance[6 * i + (j + 3)] = 0.;
      m.twist.covariance[6 * (i + 3) + j] = 0.;
    }
  }
}
}  // namespace tobas
