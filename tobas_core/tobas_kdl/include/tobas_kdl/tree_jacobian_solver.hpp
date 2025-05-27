#pragma once

#include "./jacobian.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace kdl
{
class TreeJacobianSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJacobianSolver(const Tree& tree);

  bool updateInternalDataStructures() override;

  int JntToJac(const JntArray& q, const std::string& seg_name);

  inline const Jacobian& getJacobian() const;

private:
  Jacobian J_out_;
  Frame T_total_;

  void resize();
};

inline const Jacobian& TreeJacobianSolver::getJacobian() const
{
  return J_out_;
}
}  // namespace kdl
