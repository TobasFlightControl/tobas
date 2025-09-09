#pragma once

#include "./qpsolver.hpp"

namespace quadprog
{
/**
 * @brief An Eigen wrapper of [qpOASES](https://github.com/coin-or/qpOASES)
 */
class QpOasesSolver : public QuadProgSolver
{
  using super = QuadProgSolver;

public:
  explicit QpOasesSolver();

  bool solve() override;

private:
  int nWSR_ = 10;
};
}  // namespace quadprog
