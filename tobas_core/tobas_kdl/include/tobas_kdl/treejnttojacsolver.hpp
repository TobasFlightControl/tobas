#pragma once

#include "./treesolveri.hpp"
#include "./jacobian.hpp"
#include "./jntarray.hpp"

namespace kdl
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
    return J_out_;
  }

private:
  Jacobian J_out_;
  Frame T_total_;
};
}  // namespace kdl
