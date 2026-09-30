// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_taskspace_vel_ctrl.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeTaskSpaceVelCtrl::TreeTaskSpaceVelCtrl(const Tree& _tree) : super(_tree), fk_(_tree), ik_(_tree)
{
  constexpr double kDefaultTimeConst = 0.3;  // [s]
  setLinearTimeConst(kDefaultTimeConst);
  setAngularTimeConst(kDefaultTimeConst);
}

void TreeTaskSpaceVelCtrl::updateInternalDataStructures()
{
  fk_.updateInternalDataStructures();
  ik_.updateInternalDataStructures();
}

std::expected<JntArray, std::string> TreeTaskSpaceVelCtrl::cartToJnt(const JntArray& _cur_q, const FrameMap& _tar_p)
{
  // Create target twist map.
  TwistMap tar_v;
  for (const auto& [seg_name, tar_p] : _tar_p) {
    // Compute current frame and twist.
    const auto cur_p = fk_.jntToCart(_cur_q, seg_name);
    // Compute target cartesian velocity.
    tar_v[seg_name] = gain_ * (tar_p - cur_p);
  }

  // Compute target joint velocities.
  return ik_.cartToJnt(_cur_q, tar_v);
}

void TreeTaskSpaceVelCtrl::setLinearTimeConst(const Vector& _t)
{
  assert(_t.x() > 0.0 && _t.y() > 0.0 && _t.z() > 0.0);
  gain_.linear = _t.inverse();
}

void TreeTaskSpaceVelCtrl::setAngularTimeConst(const Vector& _t)
{
  assert(_t.x() > 0.0 && _t.y() > 0.0 && _t.z() > 0.0);
  gain_.angular = _t.inverse();
}

void TreeTaskSpaceVelCtrl::setLinearTimeConst(const double& _t)
{
  assert(_t > 0.0);
  gain_.linear.fill(1.0 / _t);
}

void TreeTaskSpaceVelCtrl::setAngularTimeConst(const double& _t)
{
  assert(_t > 0.0);
  gain_.angular.fill(1.0 / _t);
}
}  // namespace kdl
}  // namespace tobas
