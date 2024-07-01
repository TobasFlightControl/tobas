#pragma once

#include <tobas_math/core.hpp>
#include <tobas_std_tools/range_holder.hpp>

#include "./treesolveri.hpp"
#include "./treefksolverpos_all.hpp"

namespace kdl
{
/* ロボットの全てのフレームを囲う最小の直方体を求める． */
class TreeBoundingBoxSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeBoundingBoxSolver(const Tree& tree);

  void updateInternalDataStructures() override;

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
  std::array<tobas_std::RangeHolder<double>, 3> ranges_;
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
  return ::sqrt(math::sqr(xWidth()) + math::sqr(yWidth()) + math::sqr(zWidth()));
}
}  // namespace kdl
