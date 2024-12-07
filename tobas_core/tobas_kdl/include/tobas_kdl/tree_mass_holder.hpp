#pragma once

#include "./tree_solver_i.hpp"

namespace kdl
{
/* Tree全体の質量のみを保持する． */
class TreeMassHolder : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeMassHolder(const Tree& tree);

  bool updateInternalDataStructures() override;

  inline const double& getMass() const
  {
    return mass_;
  }

private:
  double mass_;

  void updateTotalMass();

  /* 指定したセグメント以下の質量を返す． */
  double computeMass(const SegmentMap::const_iterator& cur_it);
};
}  // namespace kdl
