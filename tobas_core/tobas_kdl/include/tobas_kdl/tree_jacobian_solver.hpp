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

  const Jacobian& getJacobian() const
  {
    return J_out_;
  }

private:
  Jacobian J_out_;
  Frame T_total_;

  void resize();
};
}  // namespace kdl
