#include "../../include/imu_complementary_filter/utils.hpp"

using namespace Eigen;

void scaleQuaternion(double gain, Quaterniond& dq)
{
  if (dq.w() < 0.)  // 0.9
  {
    // Slerp (Spherical linear interpolation)
    double angle = acos(dq.w());
    double A = sin(angle * (1. - gain)) / sin(angle);
    double B = sin(angle * gain) / sin(angle);
    dq.w() = A + B * dq.w();
    dq.x() = B * dq.x();
    dq.y() = B * dq.y();
    dq.z() = B * dq.z();
  }
  else
  {
    // Lerp (Linear interpolation)
    dq.w() = (1. - gain) + gain * dq.w();
    dq.x() = gain * dq.x();
    dq.y() = gain * dq.y();
    dq.z() = gain * dq.z();
  }

  dq.normalize();
}
