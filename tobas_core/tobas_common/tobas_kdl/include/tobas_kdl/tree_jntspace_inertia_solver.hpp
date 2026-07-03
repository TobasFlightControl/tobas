// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./jntspace_inertia_matrix.hpp"
#include "./tree_id_solver_rne.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief Tree version of `kdl::ChainDynParam`.
 */
class TreeJntSpaceInertiaSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJntSpaceInertiaSolver(const Tree& tree);

  bool updateInternalDataStructures() override;

  /**
   * @brief Compute the joint-space inertia matrix using the unit vector method.
   *
   * @param q Joint angles.
   */
  int jntToMass(const JntArray& q);

  inline const JntSpaceInertiaMatrix& getMass() const;

private:
  TreeIdSolver_RNE rne_;

  std::vector<JntArray> elements_;
  JntSpaceInertiaMatrix H_out_;
  JntArray jntarray_null_;

  void resize();
};

inline const JntSpaceInertiaMatrix& TreeJntSpaceInertiaSolver::getMass() const
{
  return H_out_;
}
}  // namespace kdl
}  // namespace tobas
