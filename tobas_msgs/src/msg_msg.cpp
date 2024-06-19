#include <tobas_std_tools/geometry.hpp>
#include <tobas_kdl_msgs/conversion/kdl_msg.hpp>

#include "../include/tobas_msgs/conversions/msg_msg.hpp"

using namespace tobas_kdl;

namespace tobas
{
void odometryTobasToMsg(const tobas_msgs::Odometry& t, nav_msgs::Odometry& m)
{
  m.header = t.header;

  poseKDLToMsg(t.frame, m.pose.pose);
  twistKDLToMsg(t.twist, m.twist.twist);

  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      m.pose.covariance[6 * i + j] = t.position_covariance(i, j);
      m.pose.covariance[6 * (i + 3) + (j + 3)] = t.orientation_covariance(i, j);
      m.pose.covariance[6 * i + (j + 3)] = 0.;
      m.pose.covariance[6 * (i + 3) + j] = 0.;

      m.twist.covariance[6 * i + j] = t.linear_velocity_covariance(i, j);
      m.twist.covariance[6 * (i + 3) + (j + 3)] = t.angular_velocity_covariance(i, j);
      m.twist.covariance[6 * i + (j + 3)] = 0.;
      m.twist.covariance[6 * (i + 3) + j] = 0.;
    }
  }
}

void odometryMsgToTobas(const nav_msgs::Odometry& m, tobas_msgs::Odometry& t)
{
  t.header = m.header;

  poseMsgToKDL(m.pose.pose, t.frame);
  twistMsgToKDL(m.twist.twist, t.twist);

  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      t.position_covariance(i, j) = m.pose.covariance[6 * i + j];
      t.orientation_covariance(i, j) = m.pose.covariance[6 * (i + 3) + (j + 3)];

      t.linear_velocity_covariance(i, j) = m.twist.covariance[6 * i + j];
      t.angular_velocity_covariance(i, j) = m.twist.covariance[6 * (i + 3) + (j + 3)];
    }
  }
}
}  // namespace tobas
