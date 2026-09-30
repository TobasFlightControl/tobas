// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./tree_id_solver.hpp"

namespace tobas
{
namespace kdl
{
class TreeJntSpacePID : public TreeSolverI
{
  using super = TreeSolverI;

public:
  static constexpr double kDefaultStiffness = 25.0;
  static constexpr double kDefaultDamping = 10.0;

  explicit TreeJntSpacePID(const Tree& tree, const Vector& grav = Vector(0, 0, -st::kGravity));

  void updateInternalDataStructures() override;

  const JntArray& cartToJnt(
    const JntArray& cur_q,
    const JntArray& cur_qd,
    const JntArray& tar_q,
    const JntArray& tar_qd,
    const JntArray& qdd_ff);
  const JntArray&
  cartToJnt(const JntArray& cur_q, const JntArray& cur_qd, const JntArray& tar_q, const JntArray& tar_qd);

  void setStiffness(const double& kp);
  void setDamping(const double& kd);

private:
  TreeIdSolver rne_;
  JntArray q_zero_;

  double kp_ = kDefaultStiffness;
  double kd_ = kDefaultDamping;

  void resize();
};
}  // namespace kdl
}  // namespace tobas
