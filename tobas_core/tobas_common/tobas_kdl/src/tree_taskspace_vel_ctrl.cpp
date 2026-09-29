// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_taskspace_vel_ctrl.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeTaskSpaceVelCtrl::TreeTaskSpaceVelCtrl(const Tree& tree) : super(tree), fk_(tree), ik_(tree)
{
  constexpr double kDefaultTimeConst = 0.3;  // [s]
  setLinearTimeConst(Vector::Constant(kDefaultTimeConst));
  setAngularTimeConst(Vector::Constant(kDefaultTimeConst));
}

void TreeTaskSpaceVelCtrl::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_.updateInternalDataStructures();
  ik_.updateInternalDataStructures();
}

int TreeTaskSpaceVelCtrl::cartToJnt(const JntArray& cur_q, const FrameMap& tar_p)
{
  // Create target twist map.
  TwistMap tar_v;
  for (const auto& [seg_name, frame] : tar_p) {
    // Compute current frame and twist.
    if (fk_.jntToCart(cur_q, seg_name) < 0) {
      return copyError(fk_);
    }

    // Compute target cartesian velocity.
    tar_v[seg_name] = gain_ * (frame - fk_.getFrame());
  }

  // Compute target joint velocities.
  if (ik_.cartToJnt(cur_q, tar_v) < 0) {
    return copyError(ik_);
  }

  return setDefaultError(kNoError);
}

void TreeTaskSpaceVelCtrl::setLinearTimeConst(const Vector& t)
{
  assert(t.x() > 0.0 && t.y() > 0.0 && t.z() > 0.0);
  gain_.linear = t.inverse();
}

void TreeTaskSpaceVelCtrl::setAngularTimeConst(const Vector& t)
{
  assert(t.x() > 0.0 && t.y() > 0.0 && t.z() > 0.0);
  gain_.angular = t.inverse();
}

void TreeTaskSpaceVelCtrl::setLinearTimeConst(const double& t)
{
  assert(t > 0.0);
  gain_.linear.fill(1.0 / t);
}

void TreeTaskSpaceVelCtrl::setAngularTimeConst(const double& t)
{
  assert(t > 0.0);
  gain_.angular.fill(1.0 / t);
}
}  // namespace kdl
}  // namespace tobas
