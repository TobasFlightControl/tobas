#include "../include/tobas_kdl/segmentjacobian.hpp"

using namespace std;
using namespace Eigen;

namespace kdl
{
Vector6d SegmentJacobian::ravel() const
{
  Vector6d res;
  res << linear.data, angular.data;
  return res;
}
}  // namespace kdl
