#include "tobas_kdl/rigid_body_inertia.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace kdl
{
RigidBodyInertia::RigidBodyInertia(double m, const Vector& oc, const RotationalInertia& Ic) : m_(m), h_(m * oc)
{
  const auto& c_eig = oc.data;
  Matrix3d tmp = c_eig * c_eig.transpose();
  tmp.diagonal().array() -= c_eig.dot(c_eig);
  I_.data = Ic.data - m * tmp;
}

bool RigidBodyInertia::isValid(string& error_msg) const
{
  if (m_ <= 0.) {
    error_msg = "Mass must be positive.";
    return false;
  }

  if (!I_.isValid(error_msg)) {
    return false;
  }

  return true;
}

ostream& operator<<(ostream& os, const RigidBodyInertia& arg)
{
  os << "Mass: " << arg.m_ << endl;
  os << "Spatial Momentum: " << arg.h_ << endl;
  os << "Rotational Inertia: " << arg.I_;
  return os;
}
}  // namespace kdl
}  // namespace tobas
