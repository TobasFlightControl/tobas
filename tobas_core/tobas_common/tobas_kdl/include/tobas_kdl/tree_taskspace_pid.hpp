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
  explicit TreeTaskSpacePID(const Tree& tree, const Vector& grav = Vector(0, 0, -st::kGravity));

  void updateInternalDataStructures() override;

  std::expected<JntArray, std::string> cartToJnt(
    const JntArray& cur_q,
    const JntArray& cur_qd,
    const FrameMap& tar_p,
    const TwistMap& tar_v,
    const AccelMap& a_ff,
    const WrenchMap& f_ext = WrenchMap());

  void setLinearStiffness(const Vector& kp);
  void setAngularStiffness(const Vector& kp);
  void setLinearDamping(const Vector& kd);
  void setAngularDamping(const Vector& kd);
  void setLinearStiffness(const double& kp);
  void setAngularStiffness(const double& kp);
  void setLinearDamping(const double& kd);
  void setAngularDamping(const double& kd);

private:
  TreeFkSolverVel fk_;
  TreeIkSolverAcc rac_;
  TreeIdSolver rne_;

  TaskSpaceStiffness kp_;
  TaskSpaceDamping kd_;
};
}  // namespace kdl
}  // namespace tobas
