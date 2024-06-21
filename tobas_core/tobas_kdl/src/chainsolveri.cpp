#include "../include/tobas_kdl/chainsolveri.hpp"

namespace kdl
{
ChainSolverI::ChainSolverI(const Chain& chain)
  : chain_(chain), nj_(chain_.getNrOfJoints()), ns_(chain_.getNrOfSegments())
{
}

void ChainSolverI::updateInternalDataStructures()
{
  nj_ = chain_.getNrOfJoints();
  ns_ = chain_.getNrOfSegments();
}
}  // namespace kdl
