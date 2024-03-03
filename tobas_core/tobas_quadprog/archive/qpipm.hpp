#pragma once

#include "./qpsolver.hpp"

namespace ctrl
{
class QpIpmSolver : public QuadProgSolver
{
  using super = QuadProgSolver;

public:
  explicit QpIpmSolver();

  Eigen::VectorXd solveQp() override;
};
}  // namespace ctrl
