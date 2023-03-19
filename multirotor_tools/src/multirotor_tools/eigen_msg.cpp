#include "../../include/multirotor_tools/eigen_msg.hpp"

using namespace std;
using namespace Eigen;

namespace tf
{
void linVelMsgToEigen(const multirotor_msgs::LinearVelocity& m, Vector3d& e)
{
  e.x() = m.vx;
  e.y() = m.vy;
  e.z() = m.vz;
}

void linVelEigenToMsg(const Vector3d& e, multirotor_msgs::LinearVelocity& m)
{
  m.vx = e.x();
  m.vy = e.y();
  m.vz = e.z();
}
}  // namespace tf
