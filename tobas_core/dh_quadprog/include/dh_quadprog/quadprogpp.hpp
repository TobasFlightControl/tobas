#pragma once

#include <QuadProg++/Array.hh>
#undef solve

#include "./qpsolver.hpp"

namespace quadprog
{
/**
 * @brief An Eigen wrapper of [Quadprogpp](https://github.com/liuq/QuadProgpp)
 */
class QuadProgppSolver : public QuadProgSolver
{
  using super = QuadProgSolver;

public:
  explicit QuadProgppSolver();

  Eigen::VectorXd solve() override;

private:
  quadprogpp::Matrix<double> G_;
  quadprogpp::Vector<double> g0_;
  quadprogpp::Matrix<double> CE_;
  quadprogpp::Vector<double> ce0_;
  quadprogpp::Matrix<double> CI_;
  quadprogpp::Vector<double> ci0_;
  quadprogpp::Vector<double> x_;
};
}  // namespace quadprog
