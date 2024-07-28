#include "../include/tobas_kdl/treeboundingboxsolver.hpp"

namespace kdl
{
TreeBoundingBoxSolver::TreeBoundingBoxSolver(const Tree& tree) : super(tree), fk_solver_(tree)
{
}

void TreeBoundingBoxSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_solver_.updateInternalDataStructures();
}

int TreeBoundingBoxSolver::solve(const JntArray& q)
{
  if (fk_solver_.JntToCart(q) < 0)
    return copyError(fk_solver_);

  for (size_t i = 0; i < 3; ++i)
    ranges_[i].reset();

  for (const auto& [_, frame] : fk_solver_.getFrames())
    for (size_t i = 0; i < 3; ++i)
      ranges_[i].update(frame.p(i));

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
