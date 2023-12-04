#include "../include/tobas_msgs/conversions/kdl_msg.hpp"

using namespace KDL;

namespace tobas
{
void poseKDLToTobas(const Frame& k, tobas_msgs::Pose& t)
{
  t.pos = k.p;
  k.M.getRPY(t.euler.roll, t.euler.pitch, t.euler.yaw);
}

void poseTobasToKDL(const tobas_msgs::Pose& t, Frame& k)
{
  k.p = t.pos;
  k.M = t.euler.toRotation();
}
}  // namespace tobas
