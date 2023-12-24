#include "../../include/tobas_kdl/conversion/kdl_msg.hpp"

namespace KDL
{
void vectorKDLToMsg(const Vector& k, geometry_msgs::Vector3& m)
{
  m.x = k.x();
  m.y = k.y();
  m.z = k.z();
}

void vectorMsgToKDL(const geometry_msgs::Vector3& m, Vector& k)
{
  k.x(m.x);
  k.y(m.y);
  k.z(m.z);
}

void pointKDLToMsg(const Vector& k, geometry_msgs::Point& m)
{
  m.x = k.x();
  m.y = k.y();
  m.z = k.z();
}

void pointMsgToKDL(const geometry_msgs::Point& m, Vector& k)
{
  k.x(m.x);
  k.y(m.y);
  k.z(m.z);
}

void twistKDLToMsg(const Twist& k, geometry_msgs::Twist& m)
{
  vectorKDLToMsg(k.vel, m.linear);
  vectorKDLToMsg(k.rot, m.angular);
}

void twistMsgToKDL(const geometry_msgs::Twist& m, Twist& k)
{
  vectorMsgToKDL(m.linear, k.vel);
  vectorMsgToKDL(m.angular, k.rot);
}

void quaternionKDLToMsg(const Quaternion& k, geometry_msgs::Quaternion& m)
{
  m.x = k.x;
  m.y = k.y;
  m.z = k.z;
  m.w = k.w;
}

void quaternionMsgToKDL(const geometry_msgs::Quaternion& m, Quaternion& k)
{
  k.x = m.x;
  k.y = m.y;
  k.z = m.z;
  k.w = m.w;
}
}  // namespace KDL
