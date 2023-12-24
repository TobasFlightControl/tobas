#pragma once

#include "./solveri.hpp"
#include "./chain.hpp"
#include "./jntarray.hpp"

namespace KDL
{
class ChainSolverI : public SolverI
{
public:
  explicit ChainSolverI(const Chain& chain);

  virtual void updateInternalDataStructures() override;

protected:
  const Chain& chain_;
  size_t nj_;  // The number of joints in the chain
  size_t ns_;  // The number of segments in the chain

  inline bool isUpToDate() const;
};

inline bool ChainSolverI::isUpToDate() const
{
  return chain_.getNrOfJoints() == nj_ && chain_.getNrOfSegments() == ns_;
}
}  // namespace KDL
