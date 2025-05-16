#pragma once

#include "./tree_fk_solver_pos.hpp"
#include "./tree_solver_i.hpp"

namespace kdl
{
class TreeJointAxisSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJointAxisSolver(const Tree& tree);

  bool updateInternalDataStructures() override;

  /* Compute joint axis wrt. the root frame. */
  int JntToCart(const JntArray& q_in, const std::string& seg_name);

  inline const Vector& getAxis() const;

private:
  TreeFkSolverPos fk_solver_;

  Vector axis_out_;
};

inline const Vector& TreeJointAxisSolver::getAxis() const
{
  return axis_out_;
}
}  // namespace kdl
