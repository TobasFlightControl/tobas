#include "../include/dh_kdl/joint.hpp"

using namespace std;

namespace KDL
{
Joint::Joint()
{
}

Frame Joint::pose(const double& q) const
{
  switch (type)
  {
    case RotAxis:
      return Frame(Rotation::Rot2(axis_, q), origin);
    case TransAxis:
      return Frame(origin + (axis_ * (q)));
    case Fixed:
      return Frame::Identity();
    default:
      throw joint_type_ex_;
  }
}

Twist Joint::twist(const double& qd) const
{
  switch (type)
  {
    case RotAxis:
      return Twist(Vector::Zero(), axis_ * qd);
    case TransAxis:
      return Twist(axis_ * qd, Vector::Zero());
    case Fixed:
      return Twist::Zero();
    default:
      throw joint_type_ex_;
  }
}

Accel Joint::accel(const double& qdd) const
{
  switch (type)
  {
    case RotAxis:
      return Accel(Vector::Zero(), axis_ * qdd);
    case TransAxis:
      return Accel(axis_ * qdd, Vector::Zero());
    case Fixed:
      return Accel::Zero();
    default:
      throw joint_type_ex_;
  }
}

SegmentJacobian Joint::jacobian() const
{
  switch (type)
  {
    case RotAxis:
      return SegmentJacobian(Vector::Zero(), axis_);
    case TransAxis:
      return SegmentJacobian(axis_, Vector::Zero());
    case Fixed:
      return SegmentJacobian::Zero();
    default:
      throw joint_type_ex_;
  }
}
}  // end of namespace KDL
