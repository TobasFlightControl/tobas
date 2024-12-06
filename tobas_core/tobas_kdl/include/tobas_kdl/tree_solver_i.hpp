#pragma once

#include "./solver_i.hpp"
#include "./tree.hpp"

namespace kdl
{
class TreeSolverI : public SolverI
{
public:
  inline explicit TreeSolverI(const Tree& tree);

  inline virtual void updateInternalDataStructures() override;

protected:
  const Tree& tree_;
  size_t nj_;  // The number of joints in the tree
  size_t ns_;  // The number of segments in the tree

  inline bool isUpToDate() const;
};

inline TreeSolverI::TreeSolverI(const Tree& tree) : tree_(tree), nj_(tree.getNrOfJoints()), ns_(tree.getNrOfSegments())
{
}

inline void TreeSolverI::updateInternalDataStructures()
{
  nj_ = tree_.getNrOfJoints();
  ns_ = tree_.getNrOfSegments();
}

inline bool TreeSolverI::isUpToDate() const
{
  return tree_.getNrOfJoints() == nj_ && tree_.getNrOfSegments() == ns_;
}
}  // namespace kdl
