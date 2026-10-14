// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./tree_id_solver.hpp"

namespace tobas::kdl
{
class TreeJntSpacePID : public TreeSolverI
{
  using super = TreeSolverI;

public:
  static constexpr double kDefaultStiffness = 25.0;
  static constexpr double kDefaultDamping = 10.0;

  explicit TreeJntSpacePID(const Tree& _tree, const Vector& _grav = Vector(0, 0, -st::kGravity));

  void updateInternalDataStructures() override;

  const JntArray& cartToJnt(
    const JntArray& _cur_q,
    const JntArray& _cur_qd,
    const JntArray& _tar_q,
    const JntArray& _tar_qd,
    const JntArray& _qdd_ff);
  const JntArray&
  cartToJnt(const JntArray& _cur_q, const JntArray& _cur_qd, const JntArray& _tar_q, const JntArray& _tar_qd);

  void setStiffness(double _kp);
  void setDamping(double _kd);

private:
  TreeIdSolver rne_;
  JntArray q_zero_;

  double kp_ = kDefaultStiffness;
  double kd_ = kDefaultDamping;

  void resize();
};
}  // namespace tobas::kdl
