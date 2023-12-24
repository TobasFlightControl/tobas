#pragma once

#include "./treesolveri.hpp"
#include "./treefksolverpos.hpp"

namespace KDL
{
class TreeJntAxisSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJntAxisSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  /* Compute joint axis wrt. the root frame. */
  int JntToCart(const JntArray& q_in, const std::string& seg_name);

  const Vector& getAxis() const
  {
    return axis_out_;
  }

private:
  TreeFkSolverPos fk_solver_;

  Vector axis_out_;
};
}  // namespace KDL
