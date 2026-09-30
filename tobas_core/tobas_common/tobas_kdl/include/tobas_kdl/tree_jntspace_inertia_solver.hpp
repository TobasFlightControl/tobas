// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./jntspace_inertia_matrix.hpp"
#include "./tree_id_solver.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeJntSpaceInertiaSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJntSpaceInertiaSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  /**
   * @brief Compute the joint-space inertia matrix using the unit vector method.
   *
   * @param q Joint angles.
   */
  const JntSpaceInertiaMatrix& jntToMass(const JntArray& q);

private:
  TreeIdSolver rne_bias_, rne_mass_;

  size_t nj_;
  std::vector<JntArray> elements_;
  JntArray q_zero_;

  JntSpaceInertiaMatrix mass_out_;

  void resize();
};

}  // namespace kdl
}  // namespace tobas
