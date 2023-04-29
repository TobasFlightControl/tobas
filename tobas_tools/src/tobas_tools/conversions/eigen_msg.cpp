#include "../../../include/tobas_tools/conversions/eigen_msg.hpp"

using namespace std;
using namespace Eigen;

namespace tf
{
void linVelMsgToEigen(const tobas_msgs::LinearVelocity& m, Vector3d& e)
{
  e.x() = m.vx;
  e.y() = m.vy;
  e.z() = m.vz;
}

void linVelEigenToMsg(const Vector3d& e, tobas_msgs::LinearVelocity& m)
{
  m.vx = e.x();
  m.vy = e.y();
  m.vz = e.z();
}

void angVelMsgToEigen(const tobas_msgs::AngularVelocity& m, Vector3d& e)
{
  e.x() = m.wx;
  e.y() = m.wy;
  e.z() = m.wz;
}

void angVelEigenToMsg(const Vector3d& e, tobas_msgs::AngularVelocity& m)
{
  m.wx = e.x();
  m.wy = e.y();
  m.wz = e.z();
}

void eulerMsgToEigen(const tobas_msgs::Euler& m, Vector3d& e)
{
  e.x() = m.roll;
  e.y() = m.pitch;
  e.z() = m.yaw;
}

void eulerEigenToMsg(const Vector3d& e, tobas_msgs::Euler& m)
{
  m.roll = e.x();
  m.pitch = e.y();
  m.yaw = e.z();
}
}  // namespace tf
