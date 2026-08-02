// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_algorithm/range_holder.hpp>

#include "./tree_fk_solver_pos_all.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
/* Compute the minimum cuboid enclosing all frames of the robot. */
class TreeBoundingBoxSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeBoundingBoxSolver(const Tree& tree);

  bool updateInternalDataStructures() override;

  int solve(const JntArray& q);

  inline double xMin() const;
  inline double xMax() const;
  inline double xWidth() const;

  inline double yMin() const;
  inline double yMax() const;
  inline double yWidth() const;

  inline double zMin() const;
  inline double zMax() const;
  inline double zWidth() const;

  inline double diagonalLength() const;

private:
  TreeFkSolverPosAll fk_solver_;
  std::array<algo::RangeHolder<double>, 3> ranges_;
};

inline double TreeBoundingBoxSolver::xMin() const
{
  return ranges_[0].min();
}

inline double TreeBoundingBoxSolver::xMax() const
{
  return ranges_[0].max();
}

inline double TreeBoundingBoxSolver::xWidth() const
{
  return ranges_[0].range();
}

inline double TreeBoundingBoxSolver::yMin() const
{
  return ranges_[1].min();
}

inline double TreeBoundingBoxSolver::yMax() const
{
  return ranges_[1].max();
}

inline double TreeBoundingBoxSolver::yWidth() const
{
  return ranges_[1].range();
}

inline double TreeBoundingBoxSolver::zMin() const
{
  return ranges_[2].min();
}

inline double TreeBoundingBoxSolver::zMax() const
{
  return ranges_[2].max();
}

inline double TreeBoundingBoxSolver::zWidth() const
{
  return ranges_[2].range();
}

inline double TreeBoundingBoxSolver::diagonalLength() const
{
  return std::hypot(xWidth(), yWidth(), zWidth());
}
}  // namespace kdl
}  // namespace tobas
