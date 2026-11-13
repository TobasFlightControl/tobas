// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/treejnttojacsolver.hpp
// Modified by Tobas, Inc.

/*
 * TreeJntToJacSolver.hpp
 *
 *  Created on: Nov 27, 2008
 *      Author: rubensmits
 */

#pragma once

#include "./jacobian.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas::kdl
{
class TreeJacobianSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJacobianSolver(const Tree& _tree);

  void updateInternalDataStructures() override;

  const Jacobian& jntToJac(const JntArray& _q, const std::string& _seg_name);

private:
  Jacobian jac_out_;

  void resize();
};
}  // namespace tobas::kdl
