#include <dh_std_tools/map.hpp>

#include "../include/dh_kdl/treetaskspacevelctrl.hpp"

using namespace std;

namespace KDL
{
TreeTaskSpaceVelCtrl::TreeTaskSpaceVelCtrl(const Tree& tree) : super(tree), fk_(tree), ik_(tree)
{
  setLinearTimeConst(Vector::Constant(kDefaultTimeConst));
  setAngularTimeConst(Vector::Constant(kDefaultTimeConst));
}

void TreeTaskSpaceVelCtrl::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_.updateInternalDataStructures();
  ik_.updateInternalDataStructures();
}

int TreeTaskSpaceVelCtrl::CartToJnt(const JntArray& cur_q, const FrameMap& tar_p)
{
  // Create target twist map
  TwistMap tar_v;
  for (const auto& [seg_name, frame] : tar_p)
  {
    // Compute current frame and twist
    if (fk_.JntToCart(cur_q, seg_name) < 0)
      return copyError(fk_);

    // Compute target cartesian velocity
    tar_v[seg_name] = gain_ * (frame - fk_.getFrame());
  }

  // Compute target joint velocities
  if (ik_.CartToJnt(cur_q, tar_v) < 0)
    return copyError(ik_);

  return setDefaultError(E_NOERROR);
}

bool TreeTaskSpaceVelCtrl::setLinearTimeConst(const Vector& t)
{
  if (t.x() < 0 || t.y() < 0 || t.z() < 0)
    return false;

  gain_.linear = t.inverse();
  return true;
}

bool TreeTaskSpaceVelCtrl::setAngularTimeConst(const Vector& t)
{
  if (t.x() < 0 || t.y() < 0 || t.z() < 0)
    return false;

  gain_.angular = t.inverse();
  return true;
}

bool TreeTaskSpaceVelCtrl::setLinearTimeConst(const double& t)
{
  if (t < 0)
    return false;

  gain_.linear.fill(1 / t);
  return true;
}

bool TreeTaskSpaceVelCtrl::setAngularTimeConst(const double& t)
{
  if (t < 0)
    return false;

  gain_.angular.fill(1 / t);
  return true;
}
}  // namespace KDL
