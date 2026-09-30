// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

#include "./tree_fk_solver_pos.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeJointAxisSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJointAxisSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  /* Compute joint axis wrt. the root frame. */
  Vector jntToCart(const JntArray& q_in, const std::string& seg_name);

private:
  TreeFkSolverPos fk_solver_;
};

}  // namespace kdl
}  // namespace tobas
