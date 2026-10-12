// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./jacobian.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeJacobianSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJacobianSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  const Jacobian& jntToJac(const JntArray& q, const std::string& seg_name);

private:
  Jacobian jac_out_;

  void resize();
};
}  // namespace kdl
}  // namespace tobas
