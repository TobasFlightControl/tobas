#include <kdl/frames.hpp>

#include "../../include/tobas_tools/utils.hpp"

using namespace KDL;

void rotateVector(
  double roll,
  double pitch,
  double yaw,
  double xb,
  double yb,
  double zb,
  double& xa,
  double& ya,
  double& za)
{
  Rotation R_a_b = Rotation::RPY(roll, pitch, yaw);
  Vector V_b(xb, yb, zb);
  Vector V_a = R_a_b * V_b;
  xa = V_a.x();
  ya = V_a.y();
  za = V_a.z();
}
