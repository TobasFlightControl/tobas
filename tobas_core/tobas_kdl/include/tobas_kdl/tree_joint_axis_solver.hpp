#pragma once

#include "./tree_solver_i.hpp"
#include "./tree_fk_solver_pos.hpp"

namespace kdl
{
class TreeJointAxisSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJointAxisSolver(const Tree& tree);

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
}  // namespace kdl
