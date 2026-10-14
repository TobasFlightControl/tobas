// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include "./taskspace_damping.hpp"
#include "./taskspace_stiffness.hpp"
#include "./tree_fk_solver_vel.hpp"
#include "./tree_id_solver.hpp"
#include "./tree_ik_solver_acc.hpp"

namespace tobas
{
namespace kdl
{
class TreeTaskSpacePID : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeTaskSpacePID(const Tree& _tree, const Vector& _grav = Vector(0, 0, -st::kGravity));

  void updateInternalDataStructures() override;

  std::expected<JntArray, std::string> cartToJnt(
    const JntArray& _cur_q,
    const JntArray& _cur_qd,
    const FrameMap& _tar_p,
    const TwistMap& _tar_v,
    const AccelMap& _a_ff,
    const WrenchMap& _f_ext = WrenchMap());

  void setLinearStiffness(const Vector& _kp);
  void setAngularStiffness(const Vector& _kp);
  void setLinearDamping(const Vector& _kd);
  void setAngularDamping(const Vector& _kd);
  void setLinearStiffness(double _kp);
  void setAngularStiffness(double _kp);
  void setLinearDamping(double _kd);
  void setAngularDamping(double _kd);

private:
  TreeFkSolverVel fk_;
  TreeIkSolverAcc rac_;
  TreeIdSolver rne_;

  TaskSpaceStiffness kp_;
  TaskSpaceDamping kd_;
};
}  // namespace kdl
}  // namespace tobas
