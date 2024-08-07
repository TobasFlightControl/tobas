#pragma once

#include "./treejnttoinertiasolver.hpp"

namespace kdl
{
/* Tree全体の質量のみを保持する． */
class TreeMassHolder : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeMassHolder(const Tree& tree);

  void updateInternalDataStructures() override;

  inline const double& getMass() const
  {
    return inertia_solver_.getInertia().getMass();
  }

private:
  TreeJntToInertiaSolver inertia_solver_;
};
}  // namespace kdl
