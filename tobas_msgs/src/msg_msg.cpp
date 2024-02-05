#include <tobas_std_tools/geometry.hpp>
#include <tobas_kdl/conversion/kdl_msg.hpp>

#include "../include/tobas_msgs/conversions/msg_msg.hpp"

using namespace KDL;

namespace tobas
{
void transformTobasToMsg(const tobas_msgs::Pose& t, geometry_msgs::Transform& m)
{
  vectorKDLToMsg(t.pos, m.translation);
  tobas_std::eulerToQuaternion(
    t.euler.roll, t.euler.pitch, t.euler.yaw, m.rotation.x, m.rotation.y, m.rotation.z,
    m.rotation.w);
}

void transformMsgToTobas(const geometry_msgs::Transform& m, tobas_msgs::Pose& t)
{
  vectorMsgToKDL(m.translation, t.pos);
  tobas_std::quaternionToEuler(
    m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w, t.euler.roll, t.euler.pitch,
    t.euler.yaw);
}

void poseTobasToMsg(const tobas_msgs::Pose& t, geometry_msgs::Pose& m)
{
  pointKDLToMsg(t.pos, m.position);
  tobas_std::eulerToQuaternion(
    t.euler.roll, t.euler.pitch, t.euler.yaw, m.orientation.x, m.orientation.y, m.orientation.z,
    m.orientation.w);
}

void poseMsgToTobas(const geometry_msgs::Pose& m, tobas_msgs::Pose& t)
{
  pointMsgToKDL(m.position, t.pos);
  tobas_std::quaternionToEuler(
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w, t.euler.roll, t.euler.pitch,
    t.euler.yaw);
}

void odometryTobasToMsg(const tobas_msgs::Odometry& t, nav_msgs::Odometry& m)
{
  m.header = t.header;

  poseTobasToMsg(t.pose, m.pose.pose);
  twistKDLToMsg(t.twist, m.twist.twist);

  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
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

void odometryMsgToTobas(const nav_msgs::Odometry& m, tobas_msgs::Odometry& t)
{
  t.header = m.header;

  poseMsgToTobas(m.pose.pose, t.pose);
  twistMsgToKDL(m.twist.twist, t.twist);

  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      t.position_covariance[3 * i + j] = m.pose.covariance[6 * i + j];
      t.orientation_covariance[3 * i + j] = m.pose.covariance[6 * (i + 3) + (j + 3)];

      t.linear_velocity_covariance[3 * i + j] = m.twist.covariance[6 * i + j];
      t.angular_velocity_covariance[3 * i + j] = m.twist.covariance[6 * (i + 3) + (j + 3)];
    }
  }
}
}  // namespace tobas
