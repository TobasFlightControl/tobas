#include "../../../include/tobas_tools/conversions/msg_msg.hpp"

namespace tf
{
void Vector3ToLinearVelocity(const geometry_msgs::Vector3& vec, tobas_msgs::LinearVelocity& v)
{
  v.vx = vec.x;
  v.vy = vec.y;
  v.vz = vec.z;
}

void LinearVelocityToVector3(const tobas_msgs::LinearVelocity& v, geometry_msgs::Vector3& vec)
{
  vec.x = v.vx;
  vec.y = v.vy;
  vec.z = v.vz;
}

void Vector3ToAngularVelocity(const geometry_msgs::Vector3& vec, tobas_msgs::AngularVelocity& w)
{
  w.wx = vec.x;
  w.wy = vec.y;
  w.wz = vec.z;
}

void AngularVelocityToVector3(const tobas_msgs::AngularVelocity& w, geometry_msgs::Vector3& vec)
{
  vec.x = w.wx;
  vec.y = w.wy;
  vec.z = w.wz;
}

void LinearAccelToVector3(const tobas_msgs::LinearAccel& a, geometry_msgs::Vector3& vec)
{
  vec.x = a.ax;
  vec.y = a.ay;
  vec.z = a.az;
}

void Vector3ToLinearAccel(const geometry_msgs::Vector3& vec, tobas_msgs::LinearAccel& a)
{
  a.ax = vec.x;
  a.ay = vec.y;
  a.az = vec.z;
}
}  // namespace tf
