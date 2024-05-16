#pragma once

#include "./treesolveri.hpp"
#include "./jacobian.hpp"
#include "./jntarray.hpp"

namespace tobas_kdl
{
class TreeJntToJacSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJntToJacSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  int JntToJac(const JntArray& q, const std::string& seg_name);

  const Jacobian& getJacobian() const
  {
    return jac_;
  }

private:
  Jacobian jac_;
};
}  // namespace tobas_kdl
