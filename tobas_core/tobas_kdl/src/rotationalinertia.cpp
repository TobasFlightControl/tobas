#include "../include/tobas_kdl/rotationalinertia.hpp"

using namespace Eigen;

namespace kdl
{
RotationalInertia::RotationalInertia(double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz)
{
  data(0, 0) = Ixx;
  data(0, 1) = data(1, 0) = Ixy;
  data(0, 2) = data(2, 0) = Ixz;
  data(1, 1) = Iyy;
  data(1, 2) = data(2, 1) = Iyz;
  data(2, 2) = Izz;
}

RotationalInertia::RotationalInertia(const Matrix3d& data) : data(data)
{
}
}  // namespace kdl
