// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include "./taskspace_damping.hpp"
#include "./tree_fk_solver_pos.hpp"
#include "./tree_ik_solver_vel.hpp"

namespace tobas
{
namespace kdl
{
class TreeTaskSpaceVelCtrl : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeTaskSpaceVelCtrl(const Tree& _tree);

  void updateInternalDataStructures() override;

  std::expected<JntArray, std::string> cartToJnt(const JntArray& _cur_q, const FrameMap& _tar_p);

  void setLinearTimeConst(const Vector& _t);
  void setAngularTimeConst(const Vector& _t);
  void setLinearTimeConst(const double& _t);
  void setAngularTimeConst(const double& _t);

private:
  TreeFkSolverPos fk_;
  TreeIkSolverVel ik_;

  TaskSpaceDamping gain_;
};

}  // namespace kdl
}  // namespace tobas
