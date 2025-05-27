#pragma once

#include "./chain.hpp"
#include "./solver_i.hpp"

namespace kdl
{
class ChainSolverI : public SolverI
{
public:
  inline explicit ChainSolverI(const Chain& chain);

  inline virtual bool updateInternalDataStructures() override;

protected:
  const Chain& chain_;
  size_t nj_;  // The number of joints in the chain
  size_t ns_;  // The number of segments in the chain

  inline bool isUpToDate() const;
};

inline ChainSolverI::ChainSolverI(const Chain& chain)
  : chain_(chain), nj_(chain_.getNrOfJoints()), ns_(chain_.getNrOfSegments())
{
}

inline bool ChainSolverI::updateInternalDataStructures()
{
  nj_ = chain_.getNrOfJoints();
  ns_ = chain_.getNrOfSegments();

  return true;
}

inline bool ChainSolverI::isUpToDate() const
{
  return chain_.getNrOfJoints() == nj_ && chain_.getNrOfSegments() == ns_;
}
}  // namespace kdl
