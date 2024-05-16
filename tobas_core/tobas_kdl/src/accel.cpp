#include "../include/tobas_kdl/accel.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_kdl
{
Vector6d Accel::ravel() const
{
  Vector6d res;
  res << linear.data, angular.data;
  return res;
}
}  // namespace tobas_kdl
