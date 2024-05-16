#include "../include/tobas_kdl/treesolveri.hpp"

namespace tobas_kdl
{
TreeSolverI::TreeSolverI(const Tree& tree)
  : tree_(tree), nj_(tree.getNrOfJoints()), ns_(tree.getNrOfSegments())
{
}

void TreeSolverI::updateInternalDataStructures()
{
  nj_ = tree_.getNrOfJoints();
  ns_ = tree_.getNrOfSegments();
}
}  // namespace tobas_kdl
