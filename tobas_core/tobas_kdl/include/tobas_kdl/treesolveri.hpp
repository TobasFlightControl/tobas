#pragma once

#include "./solveri.hpp"
#include "./tree.hpp"

namespace kdl
{
class TreeSolverI : public SolverI
{
public:
  explicit TreeSolverI(const Tree& tree);

  virtual void updateInternalDataStructures() override;

protected:
  const Tree& tree_;
  size_t nj_;  // The number of joints in the tree
  size_t ns_;  // The number of segments in the tree

  inline bool isUpToDate() const;
};

inline bool TreeSolverI::isUpToDate() const
{
  return tree_.getNrOfJoints() == nj_ && tree_.getNrOfSegments() == ns_;
}
}  // namespace kdl
