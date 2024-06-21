#include <tobas_kdl_msgs/conversion/kdl_msg.hpp>

namespace kdl
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

void rotationKDLToMsg(const Rotation& k, geometry_msgs::Quaternion& m)
{
  k.getQuaternion(m.x, m.y, m.z, m.w);
}

void rotationMsgToKDL(const geometry_msgs::Quaternion& m, Rotation& k)
{
  k = Rotation::Quaternion(m.x, m.y, m.z, m.w);
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

void poseKDLToMsg(const Frame& k, geometry_msgs::Pose& m)
{
  pointKDLToMsg(k.p, m.position);
  rotationKDLToMsg(k.M, m.orientation);
}

void poseMsgToKDL(const geometry_msgs::Pose& m, Frame& k)
{
  pointMsgToKDL(m.position, k.p);
  rotationMsgToKDL(m.orientation, k.M);
}

void transformKDLToMsg(const Frame& k, geometry_msgs::Transform& m)
{
  vectorKDLToMsg(k.p, m.translation);
  rotationKDLToMsg(k.M, m.rotation);
}

void transformMsgToKDL(const geometry_msgs::Transform& m, Frame& k)
{
  vectorMsgToKDL(m.translation, k.p);
  rotationMsgToKDL(m.rotation, k.M);
}
}  // namespace kdl
