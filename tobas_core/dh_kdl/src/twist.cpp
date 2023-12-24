#include "../include/dh_kdl/twist.hpp"

using namespace std;
using namespace Eigen;

namespace KDL
{
Vector6d Twist::ravel() const
{
  Vector6d res;
  res << vel.data, rot.data;
  return res;
}
}  // namespace KDL
